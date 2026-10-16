#ifndef QBEAD_PIXELS_H
#define QBEAD_PIXELS_H

#include <Adafruit_NeoPixel.h>

namespace Qbead
{
    class QbeadIndexed_NeoPixelStrip : public Adafruit_NeoPixel {
    public:
      using Adafruit_NeoPixel::Adafruit_NeoPixel;   // inherit constructors
      static constexpr uint16_t FLIP = 7; // Use utils macro?

      static uint16_t map_(uint16_t n) { return n < FLIP ? (FLIP - 1 - n) : n; }

      void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
        Adafruit_NeoPixel::setPixelColor(map_(n), r, g, b);
      }
      void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
        Adafruit_NeoPixel::setPixelColor(map_(n), r, g, b, w);
      }
      void setPixelColor(uint16_t n, uint32_t c) {
        Adafruit_NeoPixel::setPixelColor(map_(n), c);
      }
      uint32_t getPixelColor(uint16_t n) const {
        return Adafruit_NeoPixel::getPixelColor(map_(n));
      }
    };

} // end namespace

#endif // QBEAD_PIXELS_H