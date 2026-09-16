// Drains the battery by holding all Qbead NeoPixels at ~85% white.
// Uses the real Qbead class / pixel indexing so it matches actual wiring.

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>
#include <bluefruit.h>
#include <Qbead.h>

Qbead::Qbead bead;

// 85% of 255 ~= 217
const uint8_t LVL = 217;

void setup() {
  // Skip bead.begin() entirely: it blocks on Serial and brings up BLE/IMU,
  // none of which we want while just draining the battery.
  bead.pixels.begin();
  bead.pixels.setBrightness(255); // use raw 0-255 color values below directly

  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, bead.pixels.Color(LVL, LVL, LVL));
  }
  bead.pixels.show();
}

void loop() {
  // Re-assert periodically in case of any driver hiccup; keeps draw constant.
  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, bead.pixels.Color(LVL, LVL, LVL));
  }
  bead.pixels.show();
  delay(5000);
}