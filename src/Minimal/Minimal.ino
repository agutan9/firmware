#include <Adafruit_NeoPixel.h>
#define PIN_NEOPIXEL 10
#define NEOPIXEL_NUM 0
Adafruit_NeoPixel pixels(NEOPIXEL_NUM, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
}

void loop() {
    Serial.println("PLEASE");
}