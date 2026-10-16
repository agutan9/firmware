#ifndef QBEAD_PIXELS_H
#define QBEAD_PIXELS_H

#include <Adafruit_NeoPixel.h>

namespace Qbead
{
    class QbeadIndexed_NeoPixelStrip : public Adafruit_NeoPixel
    {
    public:
        using Adafruit_NeoPixel::Adafruit_NeoPixel; // inherit constructors
        static constexpr uint16_t FLIP = 7;         // Use utils macro?

        static uint16_t map_(uint16_t n) { return n < FLIP ? (FLIP - 1 - n) : n; }

        void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
        {
            Adafruit_NeoPixel::setPixelColor(map_(n), r, g, b);
        }
        void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
        {
            Adafruit_NeoPixel::setPixelColor(map_(n), r, g, b, w);
        }
        void setPixelColor(uint16_t n, uint32_t c)
        {
            Adafruit_NeoPixel::setPixelColor(map_(n), c);
        }
        uint32_t getPixelColor(uint16_t n) const
        {
            return Adafruit_NeoPixel::getPixelColor(map_(n));
        }
        /*!
          @brief   Fill all or part of the NeoPixel strip with a color.
          @param   c      32-bit color value. Most significant byte is white (for
                          RGBW pixels) or ignored (for RGB pixels), next is red,
                          then green, and least significant byte is blue. If all
                          arguments are unspecified, this will be 0 (off).
          @param   first  Index of first pixel to fill, starting from 0. Must be
                          in-bounds, no clipping is performed. 0 if unspecified.
          @param   count  Number of pixels to fill, as a positive value. Passing
                          0 or leaving unspecified will fill to end of strip.
        */
        void fill(uint32_t c = 0, uint16_t first = 0, uint16_t count = 0)
        {
            uint16_t i, end;

            if (first >= numLEDs)
            {
                return; // If first LED is past end of strip, nothing to do
            }

            // Calculate the index ONE AFTER the last pixel to fill
            if (count == 0)
            {
                // Fill to end of strip
                end = numLEDs;
            }
            else
            {
                // Ensure that the loop won't go past the last pixel
                end = first + count;
                if (end > numLEDs)
                    end = numLEDs;
            }

            for (i = first; i < end; i++)
            {
                this->setPixelColor(i, c);
            }
        }

        /*!
          @brief   Fill NeoPixel strip with one or more cycles of hues.
                   Everyone loves the rainbow swirl so much, now it's canon!
          @param   first_hue   Hue of first pixel, 0-65535, representing one full
                               cycle of the color wheel. Each subsequent pixel will
                               be offset to complete one or more cycles over the
                               length of the strip.
          @param   reps        Number of cycles of the color wheel over the length
                               of the strip. Default is 1. Negative values can be
                               used to reverse the hue order.
          @param   saturation  Saturation (optional), 0-255 = gray to pure hue,
                               default = 255.
          @param   brightness  Brightness/value (optional), 0-255 = off to max,
                               default = 255. This is distinct and in combination
                               with any configured global strip brightness.
          @param   gammify     If true (default), apply gamma correction to colors
                               for better appearance.
        */
        void rainbow(uint16_t first_hue = 0, int8_t reps = 1,
               uint8_t saturation = 255, uint8_t brightness = 255,
               bool gammify = true)
        {
            for (uint16_t i = 0; i < numLEDs; i++)
            {
                uint16_t hue = first_hue + (i * reps * 65536) / numLEDs;
                uint32_t color = ColorHSV(hue, saturation, brightness);
                if (gammify)
                    color = gamma32(color);
                setPixelColor(i, color);
            }
        }
    }; 
} // end namespace
#endif // QBEAD_PIXELS_H