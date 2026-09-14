#include <Arduino.h>
#include <Wire.h>
#include <unity.h>

static bool foundLSM6DS3 = false;

void test_scan_internal_i2c_bus() {
  uint8_t deviceCount = 0;

  Serial.println();
  Serial.println("Scanning internal I2C bus: Wire1");

  for (uint8_t address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    const uint8_t error = Wire1.endTransmission();

    if (error == 0) {
      Serial.print("Found I2C device at 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);

      deviceCount++;

      if (address == 0x6A) {
        foundLSM6DS3 = true;
      }
    }
  }

  Serial.print("Number of responding devices: ");
  Serial.println(deviceCount);

  TEST_ASSERT_TRUE_MESSAGE(
    foundLSM6DS3,
    "The internal Qbead IMU did not acknowledge on Wire1 at 0x6A."
  );
}

void setup() {
  Serial.begin(9600);

  // Native USB can take several seconds to enumerate after DFU upload.
  // The PlatformIO test runner must attach before Unity prints its results.
  delay(5000);

  Wire1.begin();
  delay(100);

  UNITY_BEGIN();
  RUN_TEST(test_scan_internal_i2c_bus);
  UNITY_END();

  // Keep the firmware alive so the serial connection does not disappear
  // immediately after Unity reports its result.
  delay(3000);
}

void loop() {
  delay(1000);
}