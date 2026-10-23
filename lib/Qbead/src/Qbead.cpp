/**
 * @file Qbead.cpp
 * @brief Implementation of the Qbead controller.
 */
#include "Qbead.h"

namespace Qbead
{
  Qbead *Qbead::callbackTarget = nullptr;

  Qbead::Qbead(const uint16_t pin00, const uint16_t pixelconfig,
               const uint16_t nsections, const uint16_t nlegs,
               const uint8_t imu_addr, const uint8_t ix, const uint8_t iy,
               const uint8_t iz, const bool sx, const bool sy, const bool sz)
      : imu(LSM6DS3(I2C_MODE, imu_addr)),
        pixels(QbeadIndexed_NeoPixelStrip(nlegs * (nsections - 1) + 2, pin00, pixelconfig)),
        prMode(*this, nsections + 1),
        nsections(nsections), nlegs(nlegs),
        theta_quant(180 / nsections), phi_quant(360 / nlegs),
        ix(ix), iy(iy), iz(iz), sx(sx), sy(sy), sz(sz)
  {
  }

  void Qbead::preConfigIMU_Settings(bool gyroOn)
  {
    // TODO: Refactor readIMU() before supporting gyro-free operation.
    imu.settings.accelRange = 8;
    imu.settings.accelSampleRate = 416;
    imu.settings.accelBandWidth = 400;
    imu.settings.gyroEnabled = gyroOn;
    imu.settings.gyroSampleRate = 416;
  }

  void Qbead::postConfigIMU_Filters()
  {
    // LPF2 cutoff at ODR/100; lower cutoffs can attenuate shakes.
    imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL,
                      LSM6DS3_ACC_GYRO_LPF2_XL_EN |
                          LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_100);
  }

  void Qbead::postConfigIMU_TapDetection()
  {
    uint8_t TAP_CFG_SETTING = LSM6DS3_ACC_GYRO_TIMER_EN_ENABLED |
                              LSM6DS3_ACC_GYRO_TAP_Z_EN_ENABLED |
                              LSM6DS3_ACC_GYRO_TAP_Y_EN_ENABLED |
                              LSM6DS3_ACC_GYRO_TAP_X_EN_ENABLED;
    imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, TAP_CFG_SETTING);
    imu.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2,
                      LSM6DS3_ACC_GYRO_SHOCK_MASK & 0b11);
    uint8_t threshold_setting = 2;
    imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, threshold_setting);

    // Retained from the original: the wrapper's single/double naming is reversed.
    imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS,
                      LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP);
    imu.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG,
                      LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_ENABLED);
    pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1), tap_isr, RISING);
    Serial.println("Enabled IMU tap interrupt!");
  }

  void Qbead::begin()
  {
    callbackTarget = this;
    Serial.begin(9600);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 15000)
    {
      ;
    }

    pixels.begin();
    clear();
    setBrightness(10);
    preConfigIMU_Settings();
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
    ble.beginDualRole();
    // The BLE SoftDevice must be initialized before obtaining the hardware seed.
    seedRNG(getHardwareSeed());
  }

  void Qbead::clear()
  {
    pixels.clear();
  }

  void Qbead::show()
  {
    pixels.show();
  }

  void Qbead::setLegPixelColor(int leg, int pixel, uint32_t color)
  {
    int mappedPixelIndex = computePixelIndex(leg, pixel);
    pixels.setPixelColor(mappedPixelIndex, color);
  }

  void Qbead::setBandColor(int band, uint32_t color, bool invert)
  {
    // TODO: Support arbitrary bands and improve the inversion convention.
    if (invert)
    {
      band = nsections - band;
    }
    if (band == 0 || band == nsections)
    {
      setLegPixelColor(0, band, color);
    }
    else
    {
      for (int leg = 0; leg < nlegs; leg++)
      {
        setLegPixelColor(leg, band, color);
      }
    }
  }

  uint32_t Qbead::getLegPixelColor(int leg, int pixel)
  {
    int mappedPixelIndex = computePixelIndex(leg, pixel);
    return pixels.getPixelColor(mappedPixelIndex);
  }

  void Qbead::addLegPixelColor(int leg, int pixel, uint32_t c0)
  {
    uint32_t c1 = getLegPixelColor(leg, pixel);
    setLegPixelColor(leg, pixel, addColor(c0, c1));
  }

  void Qbead::setBrightness(uint8_t b)
  {
    pixels.setBrightness(b);
  }

  void Qbead::setBloch_deg(const BlochVector &state, uint32_t color)
  {
    setBloch_deg(state.theta, state.phi, color);
  }

  void Qbead::setBloch_deg(float theta, float phi, uint32_t color)
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
      int theta_int = min(nsections - 1, round(theta_section));
      int phi_int = round(phi / phi_quant);
      phi_int = phi_int > nlegs - 1 ? 0 : phi_int;
      setLegPixelColor(phi_int, theta_int, color);
    }
  }

  void Qbead::setBloch_deg_smooth(const BlochVector &state, uint32_t color)
  {
    setBloch_deg_smooth(state.theta, state.phi, color);
  }

  void Qbead::setBloch_deg_smooth(float theta, float phi, uint32_t c)
  {
    if (!checkThetaAndPhi(theta, phi))
      return;
    float theta_section = theta / theta_quant;
    int theta_int = min(nsections - 1, round(theta_section));
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

  void Qbead::testPixels()
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

  bool Qbead::wasTapped()
  {
    if (!tappedrecorded)
      return false;
    bool wasTapped = tapped;
    tapped = false;
    tappedrecorded = false;
    if (wasTapped)
    {
      x_whentapped = whentapped_buffer[0];
      y_whentapped = whentapped_buffer[1];
      z_whentapped = whentapped_buffer[2];
    }
    return wasTapped;
  }

  bool Qbead::wasShaken()
  {
    if (!shaken)
      return false;
    shaken = false;
    x_whenshaken = whenshaken_buffer[0];
    y_whenshaken = whenshaken_buffer[1];
    z_whenshaken = whenshaken_buffer[2];
    return true;
  }

  bool Qbead::entangle(bool localTrigger, uint32_t state)
  {
    const uint32_t now = millis();
    uint32_t receivedAtMs;
    if (ble.takeEntangleRequest(receivedAtMs))
    {
      remoteEntangleRequestPending = true;
      remoteEntangleRequestReceivedAtMs = receivedAtMs;
      Serial.println("[INFO]{ENTANGLE} Remote request queued");
    }
    if (localTrigger)
    {
      localEntangleRequestPending = true;
      localEntangleRequestStartedAtMs = now;
      ble.sendData(BLEManager::CommandType::Entangle, 0, 0, 0);
      Serial.println("[INFO]{ENTANGLE} Local tap; request sent");
    }
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
    if (localEntangleRequestPending && remoteEntangleRequestPending)
    {
      localEntangleRequestPending = false;
      remoteEntangleRequestPending = false;
      Serial.println("[INFO]{ENTANGLE} Requests matched");
      return applyPreparedState(state);
    }
    return false;
  }

  BLEManager::DataPacket Qbead::takeLatestPacket()
  {
    BLEManager::DataPacket packet;
    if (!ble.takePacket(packet))
    {
      return {BLEManager::CommandType::None, 0, 0, 0};
    }
    return packet;
  }

  void Qbead::tap_isr()
  {
    callbackTarget->tappedrecorded = false;
    callbackTarget->tapped = true;
  }

  void Qbead::readIMU(bool print)
  {
    read_buffer[0] = imu.readFloatAccelX();
    read_buffer[1] = imu.readFloatAccelY();
    read_buffer[2] = imu.readFloatAccelZ();
    rx = (1 - 2 * sx) * read_buffer[ix];
    ry = (1 - 2 * sy) * read_buffer[iy];
    rz = (1 - 2 * sz) * read_buffer[iz];
    float rawmag2 = rx * rx + ry * ry + rz * rz;

    uint32_t T_new = micros();
    uint32_t delta = T_new - T_imu;
    float dt = delta * 1e-6f;
    T_imu = T_new;

    // TODO: Allow gyro-free operation and consider exposing these tuning values.
    const float stillAccelTolerance = 0.03f;
    const float stillGyroMaxSquaredDps2 = 9.f;
    const float biasLearningGain = 0.002f;

    float gyroChipDps[3] = {
        imu.readFloatGyroX(), imu.readFloatGyroY(), imu.readFloatGyroZ()};
    float accelMagnitude = sqrtf(rawmag2);
    float gyroResidualSquaredDps2 = 0;
    for (int i = 0; i < 3; i++)
    {
      float gyroResidualDps = gyroChipDps[i] - gyroBiasDps[i];
      gyroResidualSquaredDps2 += gyroResidualDps * gyroResidualDps;
    }
    bool isStill = fabsf(accelMagnitude - 1.f) < stillAccelTolerance &&
                   gyroResidualSquaredDps2 < stillGyroMaxSquaredDps2;
    if (isStill)
      for (int i = 0; i < 3; i++)
        gyroBiasDps[i] += biasLearningGain * (gyroChipDps[i] - gyroBiasDps[i]);

    float gyroSphereDps[3] = {
        (1 - 2 * sx) * (gyroChipDps[ix] - gyroBiasDps[ix]),
        (1 - 2 * sy) * (gyroChipDps[iy] - gyroBiasDps[iy]),
        (1 - 2 * sz) * (gyroChipDps[iz] - gyroBiasDps[iz])};
    float accelSphere[3] = {rx, ry, rz};
    gravity.update(accelSphere, gyroSphereDps, dt);
    if (gravity.isInitialised &&
        shake.update(accelSphere, dt, millis(), gravity.gravityEstimate))
    {
      for (int i = 0; i < 3; i++)
        whenshaken_buffer[i] = shake.shakeAxisAtStart[i];
      shaken = true;
    }

    if (!hasAccelSample || accelSmoothingTimeUs == 0 || delta >= accelSmoothingTimeUs)
    {
      x = rx;
      y = ry;
      z = rz;
    }
    else
    {
      const float alpha = static_cast<float>(delta) / accelSmoothingTimeUs;
      x += alpha * (rx - x);
      y += alpha * (ry - y);
      z += alpha * (rz - z);
    }
    hasAccelSample = true;
    float mag2 = x * x + y * y + z * z;
    theta_accvec = theta(x, y, z) * RAD_TO_DEG;
    phi_accvec = phi(x, y) * RAD_TO_DEG;
    if (phi_accvec < 0.f)
      phi_accvec += 360.f;

    if (!tappedrecorded && tapped)
    {
      tappedrecorded = true;
      if (gravity.isInitialised)
      {
        for (int i = 0; i < 3; i++)
          whentapped_buffer[i] = gravity.gravityEstimate[i];
      }
      else
      {
        float smoothedNorm = sqrtf(x * x + y * y + z * z);
        if (smoothedNorm > 1e-6f)
        {
          whentapped_buffer[0] = x / smoothedNorm;
          whentapped_buffer[1] = y / smoothedNorm;
          whentapped_buffer[2] = z / smoothedNorm;
        }
        // Keep the previous buffer if normalization would be invalid.
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
    read_buffer[0] = x;
    read_buffer[1] = y;
    read_buffer[2] = z;
  }

  bool Qbead::hasState(const BlochVector &state,
                       float thetaTolerance, float phiTolerance) const
  {
    for (uint8_t i = 0; i < innerStateCount; i++)
    {
      const BlochVector &item = innerStates[i];
      float thetaDifference = fabsf(item.theta - state.theta);
      float phiDifference = fabsf(item.phi - state.phi);
      phiDifference = min(phiDifference, 360.0f - phiDifference);
      if (thetaDifference <= thetaTolerance && phiDifference <= phiTolerance)
      {
        return true;
      }
    }
    return false;
  }

  void Qbead::addState(BlochVector &state)
  {
    BlochVector newState(state.theta, state.phi);
    if (!hasState(newState) && innerStateCount < INNER_STATE_COUNT)
    {
      innerStates[innerStateCount++] = newState;
    }
  }

  void Qbead::clearStates()
  {
    innerStateCount = 0;
    cyclingIndex = 0;
  }

  bool Qbead::applyPreparedState(uint32_t state)
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

  void Qbead::displayCurrentStatesStatic(uint8_t colorOffset)
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

  void Qbead::displayCurrentStatesCycling()
  {
    if (innerStateCount == 0)
    {
      return;
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

  void Qbead::getBluetoothAddres(uint8_t localAddress[6])
  {
    ble.getAddr(localAddress);
  }

  void Qbead::sendData(BLEManager::CommandType type, uint32_t value,
                       uint32_t theta, uint32_t phi)
  {
    return ble.sendData(type, value, theta, phi);
  }
} // namespace Qbead
