#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>

#include <bluefruit.h>
#include <Qbead.h>

Qbead::Qbead bead;

// put function declarations here:
//int myFunction(int, int);

void setup() {
  delay(20);
  bead.begin();
  bead.setBrightness(25);
  bead.testPixels();
  delay(100);
}

void loop() {
  bead.readIMU(false);
  bead.clear();
  bead.setBloch_deg_smooth(bead.t_ble, bead.p_ble, bead.c_ble);
  bead.show();
  delay(20);
}

// put function definitions here:
//int myFunction(int x, int y) {
//  return x + y;
//}