/**
 * @file QbeadPixels.cpp
 * @brief Implementation of Qbead::QbeadIndexed_NeoPixelStrip.
 */
#include "QbeadPixels.h"

namespace Qbead
{
    uint16_t QbeadIndexed_NeoPixelStrip::map_(uint16_t n) { return n < FLIP ? (FLIP - 1 - n) : n; }

    void QbeadIndexed_NeoPixelStrip::setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
    {
        Adafruit_NeoPixel::setPixelColor(map_(n), r, g, b);
    }

    void QbeadIndexed_NeoPixelStrip::setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
    {
        Adafruit_NeoPixel::setPixelColor(map_(n), r, g, b, w);
    }

    void QbeadIndexed_NeoPixelStrip::setPixelColor(uint16_t n, uint32_t c)
    {
        Adafruit_NeoPixel::setPixelColor(map_(n), c);
    }

    uint32_t QbeadIndexed_NeoPixelStrip::getPixelColor(uint16_t n) const
    {
        return Adafruit_NeoPixel::getPixelColor(map_(n));
    }

    void QbeadIndexed_NeoPixelStrip::fill(uint32_t c, uint16_t first, uint16_t count)
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

    void QbeadIndexed_NeoPixelStrip::rainbow(uint16_t first_hue, int8_t reps,
                                             uint8_t saturation, uint8_t brightness,
                                             bool gammify)
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
} // end namespace
