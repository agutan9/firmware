#include <Arduino.h>
#include <Qbead.h>
#include "LSM6DS3.h"
#include "Wire.h"

LSM6DS3 myIMU(I2C_MODE, 0x6A);
int64_t initResult = -1;
uint8_t whoAmI = -1;

void setup() {
  Serial.begin(9600);
  delay(7000);

  Wire.begin();

  #ifdef PIN_LSM6DS3TR_C_INT1
    Serial.print("INT1 pin number: ");
    Serial.println(PIN_LSM6DS3TR_C_INT1);
    pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
    Serial.print("INT1 idle level: ");
    Serial.println(digitalRead(PIN_LSM6DS3TR_C_INT1));
  #else
    Serial.println("PIN_LSM6DS3TR_C_INT1 not defined either!");
  #endif

  whoAmI = 0;
  myIMU.readRegister(&whoAmI, 0x0F);
  Serial.print("WHO_AM_I raw value: 0x");
  Serial.println(whoAmI, HEX);

  initResult = myIMU.begin();
}

void loop() {
  Serial.print("begin(): ");
  Serial.print((long)initResult);
  Serial.print("whoAmI(): ");
  Serial.print((long)whoAmI);
  Serial.print("  Acc X: ");
  Serial.print(myIMU.readFloatAccelX());
  Serial.print("  Y: ");
  Serial.print(myIMU.readFloatAccelY());
  Serial.print("  Z: ");
  Serial.println(myIMU.readFloatAccelZ());
  delay(1000);
}