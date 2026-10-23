/**
 * @file QbeadPixels.h
 * @brief NeoPixel strip wrapper that remaps the first, reversed segment of the Qbead PCB.
 *
 * Implementations live in QbeadPixels.cpp.
 */
#ifndef QBEAD_PIXELS_H
#define QBEAD_PIXELS_H

#include <Adafruit_NeoPixel.h>

namespace Qbead
{
  /**
   * @brief Adafruit_NeoPixel strip with Qbead-specific index remapping.
   *
   * The first FLIP pixels are wired in reverse order on the PCB. Every pixel access
   * goes through map_(), so callers can use logical indices throughout.
   *
   * @note The Adafruit_NeoPixel methods are not virtual; these overloads hide the base
   *       versions and only apply when called through this type.
   */
  class QbeadIndexed_NeoPixelStrip : public Adafruit_NeoPixel
  {
  public:
    using Adafruit_NeoPixel::Adafruit_NeoPixel; /**< Inherit all Adafruit_NeoPixel constructors. */

    /** Number of leading pixels wired in reverse order. Use utils macro? */
    static constexpr uint16_t FLIP = 7;

    /**
     * @brief Maps a logical pixel index to the physical strip index.
     * @param n Logical index.
     * @return FLIP - 1 - n for n < FLIP, otherwise n.
     */
    static uint16_t map_(uint16_t n);

    /**
     * @brief Sets a pixel's colour from separate RGB channels.
     * @param n Logical pixel index.
     * @param r Red, 0-255.
     * @param g Green, 0-255.
     * @param b Blue, 0-255.
     */
    void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Sets a pixel's colour from separate RGBW channels.
     * @param n Logical pixel index.
     * @param r Red, 0-255.
     * @param g Green, 0-255.
     * @param b Blue, 0-255.
     * @param w White, 0-255 (ignored on RGB strips).
     */
    void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

    /**
     * @brief Sets a pixel's colour from a packed value.
     * @param n Logical pixel index.
     * @param c Packed colour 0xWWRRGGBB (white byte ignored on RGB strips).
     */
    void setPixelColor(uint16_t n, uint32_t c);

    /**
     * @brief Reads back a pixel's colour.
     * @param n Logical pixel index.
     * @return Packed colour as stored by Adafruit_NeoPixel (brightness-scaled).
     */
    uint32_t getPixelColor(uint16_t n) const;

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
    void fill(uint32_t c = 0, uint16_t first = 0, uint16_t count = 0);

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
                 bool gammify = true);
  };
} // end namespace
#endif // QBEAD_PIXELS_H
