#include <Arduino.h>
#include <Wire.h>
#include <unity.h>

static bool foundLSM6DS3 = false;

void test_scan_i2c_bus() {
  uint8_t deviceCount = 0;

  Serial.println();
  Serial.println("Scanning I2C bus...");

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission();

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
    "Expected the Qbead LSM6DS3 IMU at I2C address 0x6A, but it did not acknowledge."
  );
}

void setup() {
  delay(1000);

  Serial.begin(9600);
  while (!Serial && millis() < 5000) {
    delay(10);
  }

  Wire.begin();
  delay(100);

  UNITY_BEGIN();
  RUN_TEST(test_scan_i2c_bus);
  UNITY_END();
}

void loop() {
  // Tests ran once in setup(). Nothing else needed.
}