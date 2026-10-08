#ifndef QBEAD_H
#define QBEAD_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>
#include <bluefruit.h>

#include "internal/BlochVector.h"
#include "internal/QbeadUtils.h"
#include "internal/QbeadBLE.h"
#include "internal/ShakeDetector.h"
#include "internal/GravityTracker.h"

namespace Qbead
{
  class Qbead
  {
  public:
    Qbead(const uint16_t pin00 = QB_LEDPIN,
          const uint16_t pixelconfig = QB_PIXELCONFIG,
          const uint16_t nsections = QB_NSECTIONS,
          const uint16_t nlegs = QB_NLEGS,
          const uint8_t imu_addr = QB_IMU_ADDR,
          const uint8_t ix = QB_IX,
          const uint8_t iy = QB_IY,
          const uint8_t iz = QB_IZ,
          const bool sx = QB_SX,
          const bool sy = QB_SY,
          const bool sz = QB_SZ)
        : imu(LSM6DS3(I2C_MODE, imu_addr)),
          pixels(Adafruit_NeoPixel(nlegs * (nsections - 1) + 2, pin00, pixelconfig)),
          nsections(nsections),
          nlegs(nlegs),
          theta_quant(180 / nsections),
          phi_quant(360 / nlegs),
          ix(ix), iy(iy), iz(iz),
          sx(sx), sy(sy), sz(sz)
    {
    }

    static Qbead *callbackTarget; // we need a global singleton static instance because bluefruit callbacks do not support context variables -- thankfully this is fine because there is indeed only one Qbead in existence at any time

    LSM6DS3 imu;
    ShakeDetector shake;
    GravityTracker gravity;
    Adafruit_NeoPixel pixels;
    BLEManager::BLEManager ble;
    BlochVector innerStates[INNER_STATE_COUNT];
    uint8_t innerStateCount = 0;

    const uint8_t nsections;
    const uint8_t nlegs;
    const uint8_t theta_quant;
    const uint8_t phi_quant;
    const uint8_t ix, iy, iz;
    const bool sx, sy, sz;
    float rbuffer[3];
    float whentapped_buffer[3] = {0.f, 0.f, 1.f};
    float whenshaken_buffer[3] = {0.f, 0.f, 1.f};   /**< Unit shake axis (sphere frame) at the start of the last shake. */
    float gyroBiasDps[3] = {0.f, 0.f, 0.f};         /**< Learned gyro zero-rate offset (deg/s), subtracted from every reading. */
    float x_whentapped, y_whentapped, z_whentapped; /**< set when wasTapped is called */
    float x_whenshaken, y_whenshaken, z_whenshaken; /**< set when wasShaken is called */
    float theta_accvec = 0.f, phi_accvec = 0.f;
    volatile bool tapped = false;
    volatile bool tappedrecorded = false;
    bool shaken = false;
    float x = 0.f, y = 0.f, z = 0.f;
    float rx = 0.f, ry = 0.f, rz = 0.f;
    uint32_t T_imu = 0;


    bool localEntangleRequestPending = false;
    uint32_t localEntangleRequestStartedAtMs = 0;
    bool remoteEntangleRequestPending = false;
    uint32_t remoteEntangleRequestReceivedAtMs = 0;

    uint32_t stateColours[INNER_STATE_COUNT] = {
        color(0, 0, 255),   // Blue
        color(255, 0, 0),   // Red
        color(0, 255, 0),   // Green
        color(255, 255, 0), // Yellow
        color(255, 0, 255), // Magenta
        color(255, 128, 0)  // Orange
    };
    uint32_t cyclingIndex = 0;
    uint32_t lastChange = 0;
    uint32_t accelSmoothingTimeUs = 100000; // Accelerometer smoothing, set to 0 to disable
    bool hasAccelSample = false;


  
    // FUTURE: PR TODO
    // In order to actually be able to set gyroOn to false readIMU needs refactoring
    // to not use gravityTracker and let shakeDetector use its internal acc only method
    /** Driver settings, written into the registers by imu.begin(). */
    void preConfigIMU_Settings(bool gyroOn = true)
    {
      imu.settings.accelRange = 8;        // g: no clipping on shakes; tap threshold units scale with this
      imu.settings.accelSampleRate = 416; // Hz, high-performance mode
      imu.settings.accelBandWidth = 400;  // TR-C: LPF1 at ODR/2 (analog bandwidth is fixed at this ODR)
      imu.settings.gyroEnabled = gyroOn;  // required by the gravity tracker
      imu.settings.gyroSampleRate = 416;
      // imu.settings.gyroRange    = 1000;  // optional: finer resolution than the 2000 dps default
    }

    // PR TODO:
    // FUTURE: add switch case for different ODR_X bandwidhts?
    // Seems silly but is general to both tap and shake and there are many more filter registers which we currently don't set
    void postConfigIMU_Filters()
    {
      // Enable low pass filter and set cutoff frequency to datarate/100. Smaller cut-offs might attenuate shake detection.
      imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, LSM6DS3_ACC_GYRO_LPF2_XL_EN | LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_100);
    }

    // Sets the main isr interrupt
    void postConfigIMU_TapDetection()
    {
      // Enable tap detection in X,Y,Z:
      uint8_t TAP_CFG_SETTING = LSM6DS3_ACC_GYRO_TIMER_EN_ENABLED | LSM6DS3_ACC_GYRO_TAP_Z_EN_ENABLED | LSM6DS3_ACC_GYRO_TAP_Y_EN_ENABLED | LSM6DS3_ACC_GYRO_TAP_X_EN_ENABLED;
      imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, TAP_CFG_SETTING);

      // Set shock time window:
      imu.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, LSM6DS3_ACC_GYRO_SHOCK_MASK & 0b11);

      // Set tap threshold:
      uint8_t threshold_setting = 2; // number between 0 and 31
      imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, threshold_setting);

      // Only do single tap detection. Seems like the naming is incorrect?
      // TODO: INDEED INCORRECT, Library wrapper implemented reverse from spec sheet
      imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP);

      // Single-tap interrupt driven to pin 1
      imu.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_ENABLED);

      // Setup interrupt callback
      pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
      attachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1), tap_isr, RISING);

      Serial.println("Enabled IMU tap interrupt!");
    }

    void begin()
    {
      callbackTarget = this;
      Serial.begin(9600);
      // Wait for Serial to become available or hit a time-out before continuing
      unsigned long t0 = millis();
      while (!Serial && millis() - t0 < 15000)
      {
        ;
      }

      pixels.begin();
      clear();
      setBrightness(10);

      preConfigIMU_Settings(); // fills imu.settings; must run before imu.begin()

      Serial.println("[INFO] Booting... Qbead on XIAO BLE Sense + LSM6DS3T-RC compiled on " __DATE__ " at " __TIME__);
      if (!imu.begin())
      {
        Serial.println("[DEBUG]{IMU} IMU initialized correctly");
      }
      else
      {
        Serial.println("[ERROR]{IMU} IMU failed to initialize");
      }

      postConfigIMU_Filters();
      postConfigIMU_TapDetection();
      // postConfigIMU_ShakeDetection: Shake is software based. 
      // Only relevant registers are the LPF's due to high-freq acc attenuation

      ble.beginDualRole();
    }

    void clear()
    {
      pixels.clear();
    }

    void show()
    {
      pixels.show();
    }

    void setLegPixelColor(int leg, int pixel, uint32_t color)
    {
      int mappedPixelIndex = computePixelIndex(leg, pixel);
      pixels.setPixelColor(mappedPixelIndex, color);
    }

    uint32_t getLegPixelColor(int leg, int pixel)
    {
      int mappedPixelIndex = computePixelIndex(leg, pixel);
      return pixels.getPixelColor(mappedPixelIndex);
    }

    void addLegPixelColor(int leg, int pixel, uint32_t c0)
    {
      uint32_t c1 = getLegPixelColor(leg, pixel);
      setLegPixelColor(leg, pixel, addColor(c0, c1));
    }

    void setBrightness(uint8_t b)
    {
      pixels.setBrightness(b);
    }

    void setBloch_deg(const BlochVector &state, uint32_t color)
    {
      setBloch_deg(state.theta, state.phi, color);
    }

    void setBloch_deg(float theta, float phi, uint32_t color)
    {
      float theta_section = theta / theta_quant;
      if (theta_section < 0.5)
      {
        setLegPixelColor(0, 0, color);
      }
      else if (theta_section > nsections - 0.5)
      {
        setLegPixelColor(0, nsections, color);
      }
      else
      {
        int theta_int = min(nsections - 1, round(theta_section)); // to avoid precision issues near the end of the range
        int phi_int = round(phi / phi_quant);
        phi_int = phi_int > nlegs - 1 ? 0 : phi_int;
        setLegPixelColor(phi_int, theta_int, color);
      }
    }

    /*
    // TODO needs brightness correction for when we have many LEDs on (when they are denser)
    // TODO make sure you are not in situations where no LEDs are lit because none are close (an edge case of brightness correction above)
    // TODO skip far-away pixels so the loop is not so expensive
    void setBloch_deg_smooth(const BlochVector& state, uint32_t color) {
      float width = 40;

      for (int phi_pixel = 0; phi_pixel <= nlegs; ++phi_pixel) {
        for (int theta_pixel = 0; theta_pixel <= nsections; ++theta_pixel) {
          float internal_angle = state.centralAngle(BlochVector(theta_quant * theta_pixel, phi_quant * phi_pixel));
          if (internal_angle > width) {
            continue;
          }

          float brightness = 1 - internal_angle * internal_angle / (width * width);

          addLegPixelColor(phi_pixel, theta_pixel, scaleColor(brightness, color));
        }
      }
    }

    void setBloch_deg_smooth(float theta, float phi, uint32_t color) {
      setBloch_deg_smooth(BlochVector(theta, phi), color);
    }
    */

    void setBloch_deg_smooth(const BlochVector &state, uint32_t color)
    {
      setBloch_deg_smooth(state.theta, state.phi, color);
    }

    void setBloch_deg_smooth(float theta, float phi, uint32_t c)
    {
      if (!checkThetaAndPhi(theta, phi))
        return;
      float theta_section = theta / theta_quant;
      int theta_int = min(nsections - 1, round(theta_section)); // to avoid precision issues near the end of the range
      int phi_int = round(phi / phi_quant);
      phi_int = phi_int > nlegs - 1 ? 0 : phi_int;

      float p = (theta_section - theta_int);
      int theta_direction = sign(p);
      p = abs(p);
      float q = 1 - p;
      p = p * p;
      q = q * q;

      uint8_t rc = redch(c);
      uint8_t gc = greench(c);
      uint8_t bc = bluech(c);

      setLegPixelColor(phi_int, theta_int, color(q * rc, q * gc, q * bc));
      setLegPixelColor(phi_int, theta_int + theta_direction, color(p * rc, p * gc, p * bc));
    }

    void testPixels()
    {
      Serial.println("[INFO] Testing all pixels discretely");
      for (int i = 0; i < pixels.numPixels(); i++)
      {
        pixels.setPixelColor(i, color(255, 255, 255));
        pixels.show();
        delay(5);
      }
      Serial.println("[INFO] Testing smooth transition between pixels");
      for (int phi = 0; phi < 360; phi += 30)
      {
        for (int theta = 0; theta < 180; theta += 6)
        {
          clear();
          setBloch_deg_smooth(theta, phi, colorWheel_deg(phi));
          show();
        }
      }
    }

    bool wasTapped()
    {
      if (!tappedrecorded)
        return false;
      // return true if IMU detected a tap since the last time wasTapped() was called
      bool wasTapped = tapped;
      // set tapped to false, so that the next time this function is called, it will return false
      tapped = false;
      tappedrecorded = false;
      // save tapped location
      if (wasTapped)
      {
        // buffer already holds sphere-frame values; don't need ix/iy/iz
        x_whentapped = whentapped_buffer[0];
        y_whentapped = whentapped_buffer[1];
        z_whentapped = whentapped_buffer[2];
      }
      return wasTapped;
    }

    // localTrigger indicates the gesture for firing a local entanglement attempt
    bool entangle(bool localTrigger, uint32_t state)
    {
      const uint32_t now = millis();

      // Capture a newly received remote request.
      uint32_t receivedAtMs;
      if (ble.takeEntangleRequest(receivedAtMs))
      {
        remoteEntangleRequestPending = true;
        remoteEntangleRequestReceivedAtMs = receivedAtMs;

        Serial.println("[INFO]{ENTANGLE} Remote request queued");
      }

      // Capture a newly detected local tap and send our request.
      if (localTrigger)
      {
        localEntangleRequestPending = true;
        localEntangleRequestStartedAtMs = now;

        ble.sendData(BLEManager::CommandType::Entangle, 0, 0, 0);

        Serial.println("[INFO]{ENTANGLE} Local tap; request sent");
      }

      // Drop expired requests.
      if (localEntangleRequestPending &&
          (uint32_t)(now - localEntangleRequestStartedAtMs) > ENTANGLE_WINDOW_MS)
      {
        localEntangleRequestPending = false;
        Serial.println("[INFO]{ENTANGLE} Local request timed out");
      }

      if (remoteEntangleRequestPending &&
          (uint32_t)(now - remoteEntangleRequestReceivedAtMs) > ENTANGLE_WINDOW_MS)
      {
        remoteEntangleRequestPending = false;
        Serial.println("[INFO]{ENTANGLE} Remote request timed out");
      }

      // Entangle only if both requests are presently valid.
      if (localEntangleRequestPending && remoteEntangleRequestPending)
      {
        localEntangleRequestPending = false;
        remoteEntangleRequestPending = false;

        Serial.println("[INFO]{ENTANGLE} Requests matched");
        return applyPreparedState(state);
      }

      return false;
    }

    BLEManager::DataPacket takeLatestPacket()
    {
      BLEManager::DataPacket packet;
      if (!ble.takePacket(packet))
      {
        return {BLEManager::CommandType::None, 0, 0, 0};
      }
      return packet;
    }
    
    bool wasShaken()
    {
      if (!shaken)
        return false;
      shaken = false;
      x_whenshaken = whenshaken_buffer[0];
      y_whenshaken = whenshaken_buffer[1];
      z_whenshaken = whenshaken_buffer[2];
      return true;
    }

    BLEManager::DataPacket takeLatestPacket()
    {
      BLEManager::DataPacket packet;
      if (!ble.takePacket(packet))
      {
        return {BLEManager::CommandType::None, 0, 0, 0};
      }
      return packet;
    }

    BLEManager::DataPacket takeLatestPacket()
    {
      BLEManager::DataPacket packet;
      if (!ble.takePacket(packet))
      {
        return {BLEManager::CommandType::None, 0, 0, 0};
      }
      return packet;
    }

    static void tap_isr()
    {
      // This function is called when the IMU triggers an interrupt. That is: when a tap is detected!
      // We have to refer to the callbackTarget of the qbead here,
      // because the one and only qbead object does not exist when this isr is defined.
      // Then readout XYZ immediately
      // We could choose to only readout XYZ when we haven't yet processed the last tap,
      // but for now, let's just update the position everytime we tap.
      callbackTarget->tappedrecorded = false;
      callbackTarget->tapped = true;
    }

    void readIMU(bool print = true)
    {
      rbuffer[0] = imu.readFloatAccelX();
      rbuffer[1] = imu.readFloatAccelY();
      rbuffer[2] = imu.readFloatAccelZ();
      rx = (1 - 2 * sx) * rbuffer[ix];
      ry = (1 - 2 * sy) * rbuffer[iy];
      rz = (1 - 2 * sz) * rbuffer[iz];
      float rawmag2 = rx * rx + ry * ry + rz * rz;

      uint32_t T_new = micros();
      uint32_t delta = T_new - T_imu;
      float dt = delta * 1e-6f;
      T_imu = T_new;

      // TODO: Should these be public class members? Possibly
      // TODO: FUTURE:
      //    Refactor such that gyroscope can (shake and gravity) can be turned off
      // Process Gyro: read in the chip frame, learn the zero-rate offset while still, then map to the sphere frame.
      const float stillAccelTolerance = 0.03f;   // |accel| within this many g of 1 g counts as still
      const float stillGyroMaxSquaredDps2 = 9.f; // (3 deg/s)^2: rotation below this counts as still
      const float biasLearningGain = 0.002f;     // fraction of the error absorbed per call

      float gyroChipDps[3] = {imu.readFloatGyroX(), imu.readFloatGyroY(), imu.readFloatGyroZ()}; // deg/s, chip frame
      float accelMagnitude = sqrtf(rawmag2);                                                     // unit of g
      float gyroResidualSquaredDps2 = 0;                                                         // squared rate left after bias removal
      for (int i = 0; i < 3; i++)
      {
        float gyroResidualDps = gyroChipDps[i] - gyroBiasDps[i];
        gyroResidualSquaredDps2 += gyroResidualDps * gyroResidualDps;
      }
      bool isStill = fabsf(accelMagnitude - 1.f) < stillAccelTolerance &&
                     gyroResidualSquaredDps2 < stillGyroMaxSquaredDps2;
      if (isStill) // learn the zero-rate offset
        for (int i = 0; i < 3; i++)
          gyroBiasDps[i] += biasLearningGain * (gyroChipDps[i] - gyroBiasDps[i]);

      float gyroSphereDps[3] = {(1 - 2 * sx) * (gyroChipDps[ix] - gyroBiasDps[ix]),
                                (1 - 2 * sy) * (gyroChipDps[iy] - gyroBiasDps[iy]),
                                (1 - 2 * sz) * (gyroChipDps[iz] - gyroBiasDps[iz])};
      float accelSphere[3] = {rx, ry, rz}; // g, sphere frame, unsmoothed
      // Track gravity and check for shakes
      gravity.update(accelSphere, gyroSphereDps, dt); // handles gaps and (re)seeding itself
      if (gravity.isInitialised &&
          shake.update(accelSphere, dt, millis(), gravity.gravityEstimate))
      {
        for (int i = 0; i < 3; i++)
          whenshaken_buffer[i] = shake.shakeAxisAtStart[i];
        shaken = true;
      }

      // Only activate smoothing filter if read gap is small enough
      if (!hasAccelSample ||
          accelSmoothingTimeUs == 0 ||
          delta >= accelSmoothingTimeUs)
      {
        x = rx;
        y = ry;
        z = rz;
      }
      else
      {
        const float alpha =
            static_cast<float>(delta) / accelSmoothingTimeUs;

        x += alpha * (rx - x);
        y += alpha * (ry - y);
        z += alpha * (rz - z);
      }

      hasAccelSample = true;
      float mag2 = x * x + y * y + z * z;

      // Calculate (display) angles from the smoothed sphere-frame acceleration,
      theta_accvec = theta(x, y, z) * RAD_TO_DEG;
      phi_accvec = phi(x, y) * RAD_TO_DEG;

      // Wrap azimuth into [0, 360).
      if (phi_accvec < 0)
      {
        phi_accvec += 360;
      }
      
      if (!tappedrecorded && tapped)
      {
        tappedrecorded = true;
        if (gravity.isInitialised)
        {
          // Best estimate: unit length, no lag during rotation, tap impulse ignored by the trust gate.
          for (int i = 0; i < 3; i++)
            whentapped_buffer[i] = gravity.gravityEstimate[i];
        }
        else
        {
          // Tracker not valid yet (boot, long gap, or continuous motion): fall back to the
          // smoothed accelerometer vector, normalised to unit length like the tracker's output.
          float smoothedNorm = sqrtf(x * x + y * y + z * z);
          if (smoothedNorm > 1e-6f)
          {
            whentapped_buffer[0] = x / smoothedNorm;
            whentapped_buffer[1] = y / smoothedNorm;
            whentapped_buffer[2] = z / smoothedNorm;
          }
          // else: keep the previous buffer content rather than storing an invalid vector
        }
      }

      if (print)
      {
        Serial.print(tappedrecorded);
        Serial.print("\t");
        Serial.print(tapped);
        Serial.print("\t");
        Serial.print(x);
        Serial.print("\t");
        Serial.print(y);
        Serial.print("\t");
        Serial.print(z);
        Serial.print("\t");
        Serial.print(mag2);
        Serial.print("\t");
        Serial.print(rawmag2);
        Serial.print("\t-1\t1\t");
        Serial.print(theta_accvec);
        Serial.print("\t");
        Serial.print(phi_accvec);
        Serial.print("\t0\t360\t");
        Serial.println();
      }

      rbuffer[0] = x;
      rbuffer[1] = y;
      rbuffer[2] = z;
    }

    bool hasState(const BlochVector &state,
                  float thetaTolerance = 1.0f,
                  float phiTolerance = 1.0f) const
    {
      for (uint8_t i = 0; i < innerStateCount; i++)
      {
        const BlochVector &item = innerStates[i];

        float thetaDifference = fabsf(item.theta - state.theta);

        float phiDifference = fabsf(item.phi - state.phi);
        phiDifference = min(phiDifference, 360.0f - phiDifference);

        if (thetaDifference <= thetaTolerance &&
            phiDifference <= phiTolerance)
        {
          return true;
        }
      }

      return false;
    }

    void addState(BlochVector &state)
    {
      BlochVector newState(state.theta, state.phi);
      if (!hasState(newState) && innerStateCount < INNER_STATE_COUNT)
      {
        innerStates[innerStateCount++] = newState;
      }
    }

    void clearStates()
    {
      innerStateCount = 0;
    }

    bool applyPreparedState(uint32_t state)
    {
      BlochVector up(0, 0);
      BlochVector down(180, 0);
      if (state == 1)
      {
        this->clearStates();
        this->addState(up);
        this->addState(down);
        return true;
      }
      else if (state == 2)
      {
        this->clearStates();
        this->addState(down);
        this->addState(up);
        return true;
      }
      return false;
    }

    // Hacky solution for the colours as it gives away inner implementation of the class
    void displayCurrentStatesStatic(uint8_t colorOffset = 0)
    {
      this->clear();
      for (uint8_t i = 0; i < innerStateCount; i++)
      {
        const BlochVector &item = innerStates[i];
        const uint32_t &itemColour = stateColours[(i + colorOffset) % INNER_STATE_COUNT];

        this->setBloch_deg(item, itemColour);
      }
      this->show();
    }

    void displayCurrentStatesCycling()
    {
      if (innerStateCount == 0)
      {
        const BlochVector &item = innerStates[i];

        float thetaDifference = fabsf(item.theta - state.theta);

        float phiDifference = fabsf(item.phi - state.phi);
        phiDifference = min(phiDifference, 360.0f - phiDifference);

        if (thetaDifference <= thetaTolerance &&
            phiDifference <= phiTolerance)
        {
          return true;
        }
      }
      uint32_t currentTime = millis();
      uint32_t deltaTime = currentTime - lastChange;
      if (deltaTime < CYCLING_TIME)
      {
        return;
      }
      this->clear();
      lastChange = currentTime;
      this->setBloch_deg(innerStates[cyclingIndex], stateColours[cyclingIndex]);
      this->show();
      cyclingIndex = (cyclingIndex + 1) % (innerStateCount);
    }
  }; // end class

  Qbead *Qbead::callbackTarget = nullptr;

} // end namespace

#endif // QBEAD_H