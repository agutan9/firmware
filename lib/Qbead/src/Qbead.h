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

  enum class CommandType : uint8_t
  {
    None = 0,            // Reserved: no packet / empty packet buffer.
    SetOrchestrator = 1, // Set or announce the entanglement orchestrator.
    Bell0 = 2,           // Request Bell state (|00> + |11>) / sqrt(2).
    Bell1 = 3            // Request Bell state (|01> + |10>) / sqrt(2).
  };

  class Qbead
  {
  public:
    Qbead(BLEManager::Role role = BLEManager::Role::Dual,
          const uint16_t pin00 = QB_LEDPIN,
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
          role(role)
    {
    }

    static Qbead *callbackTarget; // we need a global singleton static instance because bluefruit callbacks do not support context variables -- thankfully this is fine because there is indeed only one Qbead in existence at any time

    LSM6DS3 imu;
    Adafruit_NeoPixel pixels;
    BLEManager::BLEManager ble;
    BLEManager::Role role;
    uint8_t isOrchastratorSet = 0; // 0 no, 1 yes I am, 2 yes but its not me

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
    float T_imu;                                    // last update from the IMU
    bool tapped = false;
    bool tappedrecorded = false;

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

    // Should use IMU specific reset pin
    bool resetIMU(LSM6DS3 &imu) {
      // CTRL3_C = 0x12, SW_RESET = bit 0
      uint8_t ctrl3c;
      if (imu.readRegister(&ctrl3c, LSM6DS3_ACC_GYRO_CTRL3_C) != IMU_SUCCESS) return false;
      imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, ctrl3c | 0x01);
      delay(1);  // datasheet: reset completes within ~50us, self-clears
      return imu.begin() == IMU_SUCCESS;
    }

    void begin()
    {
      callbackTarget = this;
      Serial.begin(9600);
      //while (!Serial)
      //  ; // TODO some form of warning or a way to give up if Serial never becomes available
      unsigned long t0 = millis();
      while (!Serial && millis() - t0 < 15000) { ; }

      pixels.begin();
      clear();
      setBrightness(10);

      Serial.println("[INFO] Booting... Qbead on XIAO BLE Sense + LSM6DS3 compiled on " __DATE__ " at " __TIME__);
      //if (!imu.begin()) // TODO resetIMU(imu) instead?
      // Attempt to force reset the IMU before init.. Needs I2C bus to work
      if (!resetIMU(imu)) // TODO revert back?
      {
        Serial.println("[DEBUG]{IMU} IMU initialized correctly");
      }
      else
      {
        Serial.println("[ERROR]{IMU} IMU failed to initialize");
      }

      setupIMUTapDetection();

      if (role == BLEManager::Role::Peripheral)
      {
        ble.beginPeripheral();
      }
      else if (role == BLEManager::Role::Central)
      {
        ble.beginCentral();
      }
      else
      {
        ble.beginDualRole();
      }
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

    bool takeTapReceived()
    {
      BLEManager::DataPacket packet;
      if (!ble.takePacket(packet))
      {
        return false;
      }
      return packet.type == 1 && packet.value == 1;
    }

    BLEManager::DataPacket takeLatestPacket()
    {
      BLEManager::DataPacket packet;
      if (!ble.takePacket(packet))
      {
        return {0, 0};
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

      float T_new = micros();
      float delta = T_new - T_imu;
      T_imu = T_new;
      const float T = 100000; // 100 ms // TODO make the filter timeconstant configurable
      if (delta > 100000)
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

      if (!tappedrecorded && tapped)
      {
        tappedrecorded = true;
        whentapped_buffer[0] = imu.readFloatAccelX();
        whentapped_buffer[1] = imu.readFloatAccelY();
        whentapped_buffer[2] = imu.readFloatAccelZ();
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

    void orchastrate()
    {
      if (isOrchastratorSet)
      {
        return;
      }
      isOrchastratorSet = 1;
      ble.sendData(static_cast<uint8_t>(CommandType::SetOrchestrator), 2); // set me orchastrator
    }

    void setOrchestrate(uint8_t newValue)
    {
      isOrchastratorSet = newValue;
    }
  }; // end class

  Qbead *Qbead::callbackTarget = nullptr;

} // end namespace

#endif // QBEAD_H