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

namespace Qbead
{
    // TODO: REMOVE AND REFACTOR
  struct ShakeDetector {
    float g[3]; bool init = false;
    uint32_t tPos = 0, tNeg = 0, tFire = 0;
    bool havePos = false, haveNeg = false;

    bool update(const float r[3], float dt, uint32_t nowMs) {
      const float TAU = 1.0f, THR = 0.8f, PERP_RATIO = 0.6f;
      const uint32_t WINDOW_MS = 300, COOLDOWN_MS = 500;
      if (!init) { memcpy(g, r, sizeof g); init = true; }

      float gm = sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
      float h[3]   = {g[0]/gm, g[1]/gm, g[2]/gm};
      float lin[3] = {r[0]-g[0], r[1]-g[1], r[2]-g[2]};
      float par  = lin[0]*h[0] + lin[1]*h[1] + lin[2]*h[2];
      float pe[3] = {lin[0]-par*h[0], lin[1]-par*h[1], lin[2]-par*h[2]};
      float perp = sqrtf(pe[0]*pe[0] + pe[1]*pe[1] + pe[2]*pe[2]);

      float a = dt / ((havePos || haveNeg) ? 4*TAU : TAU);   // slow down mid-shake
      for (int i = 0; i < 3; i++) g[i] += a * (r[i] - g[i]);

      if (fabsf(par) > THR && perp < PERP_RATIO * fabsf(par)) {
        if (par > 0) { tPos = nowMs; havePos = true; }
        else         { tNeg = nowMs; haveNeg = true; }
      }
      if (havePos && nowMs - tPos > WINDOW_MS) havePos = false;
      if (haveNeg && nowMs - tNeg > WINDOW_MS) haveNeg = false;

      if (havePos && haveNeg && nowMs - tFire > COOLDOWN_MS) {
        havePos = haveNeg = false; tFire = nowMs; return true;
      }
      return false;
    }
  };

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
          sx(sx), sy(sy), sz(sz),
          // TODO: REFACTOR
          shake()
    {
    }

    static Qbead *callbackTarget; // we need a global singleton static instance because bluefruit callbacks do not support context variables -- thankfully this is fine because there is indeed only one Qbead in existence at any time

    LSM6DS3 imu;
    ShakeDetector shake; // TODO
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
    float whentapped_buffer[3];
    float x_whentapped, y_whentapped, z_whentapped; // set when wasTapped is called
    float x, y, z, rx, ry, rz;                      // filtered and raw acc, in units of g
    float t_acc, p_acc;                             // theta and phi according to gravity
    //float T_imu;                                    // last update from the IMU TODO
    uint32_t T_imu;
    bool tapped = false;
    bool tappedrecorded = false;
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

    void setupIMUTapDetection()
    {
      // Turn on the accelerometer
      // Acc = 416Hz (High-Performance mode)
      imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, LSM6DS3_ACC_GYRO_ODR_XL_416Hz);

      // Optionally, disable gyroscope to save power
      // imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN);

      // Enable tap detection in X,Y,Z:
      uint8_t TAP_CFG_SETTING = LSM6DS3_ACC_GYRO_TIMER_EN_ENABLED | LSM6DS3_ACC_GYRO_TAP_Z_EN_ENABLED | LSM6DS3_ACC_GYRO_TAP_Y_EN_ENABLED | LSM6DS3_ACC_GYRO_TAP_X_EN_ENABLED;
      imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, TAP_CFG_SETTING);

      // Set shock time window:
      imu.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, LSM6DS3_ACC_GYRO_SHOCK_MASK & 0b11);

      // Set tap threshold:
      uint8_t thrshold_setting = 8; // number between 0 and 31
      imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, thrshold_setting);

      // Only do single tap detection. Seems like the naming is incorrect?
      // TODO: INDEED INCORRECT, Library wrapper implemented reverse from spec sheet
      imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP);

      // Single-tap interrupt driven to pin 1
      imu.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_ENABLED);

      // Enable low pass filter and set cutoff frequency to datarate/400
      imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, LSM6DS3_ACC_GYRO_LPF2_XL_EN | LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_400);

      // Setup interrupt callback
      pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
      attachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1), tap_isr, RISING);

      Serial.println("Enabled IMU interrupt!");
    }

    void begin()
    {
      callbackTarget = this;
      Serial.begin(9600);
      // while (!Serial)
      //   ; // TODO some form of warning or a way to give up if Serial never becomes available
      unsigned long t0 = millis();
      while (!Serial && millis() - t0 < 15000)
      {
        ;
      }

      pixels.begin();
      clear();
      setBrightness(10);

      Serial.println("[INFO] Booting... Qbead on XIAO BLE Sense + LSM6DS3 compiled on " __DATE__ " at " __TIME__);
      if (!imu.begin())
      {
        Serial.println("[DEBUG]{IMU} IMU initialized correctly");
      }
      else
      {
        Serial.println("[ERROR]{IMU} IMU failed to initialize");
      }

      setupIMUTapDetection();

      // TODO
      uint8_t id; imu.readRegister(&id, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
      Serial.println(id, HEX);
      // TODO

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
        x_whentapped = whentapped_buffer[ix];
        y_whentapped = whentapped_buffer[iy];
        z_whentapped = whentapped_buffer[iz];
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

    BLEManager::DataPacket takeLatestPacket()
    {
      BLEManager::DataPacket packet;
      if (!ble.takePacket(packet))
      {
        return {BLEManager::CommandType::None, 0};
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

      uint32_t T_new = micros(); //TODO: Can lose resolution after ~3-4h
      uint32_t delta = T_new - T_imu;
      T_imu = T_new;
      const float T = 100000; // 100 ms // TODO make the filter timeconstant configurable
      if (delta > T)
      {
        x = rx;
        y = ry;
        z = rz;
      }
      else
      {
        float d = delta / T;
        x = d * rx + (1 - d) * x;
        y = d * ry + (1 - d) * y;
        z = d * rz + (1 - d) * z;
      }
      float mag2 = x * x + y * y + z * z;

      t_acc = theta(x, y, z) * 180 / 3.14159;
      p_acc = phi(x, y) * 180 / 3.14159;
      if (p_acc < 0)
      {
        p_acc += 360;
      } // to bring it to [0,360] range

      // TODO
      float r[3];
      r[0] = rx;
      r[1] = rx;
      r[2] = rx;
      if (shake.update(r, delta*1e-6f, T_new)){
        Serial.println("SHAKEN");
      }
      // TODO

      if (!tappedrecorded && tapped)
      {
        tappedrecorded = true;
        // TODO: Why re-read this info without it being ex-LPF'ed???
        //whentapped_buffer[0] = imu.readFloatAccelX();
        //whentapped_buffer[1] = imu.readFloatAccelY();
        //whentapped_buffer[2] = imu.readFloatAccelZ();

        whentapped_buffer[0] = x;
        whentapped_buffer[1] = y;
        whentapped_buffer[2] = z;
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
        Serial.print(t_acc);
        Serial.print("\t");
        Serial.print(p_acc);
        Serial.print("\t-360\t360\t");
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