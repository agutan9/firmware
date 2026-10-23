/**
 * @file ProbabilityDisplay.h
 * @brief "Liquid fill" visualisations of measurement statistics on the Qbead LED bands.
 *
 * Implementations live in ProbabilityDisplay.cpp.
 */
#ifndef QBEAD_PROB_DISPLAY_H
#define QBEAD_PROB_DISPLAY_H

#include <stdint.h>
#include "QbeadUtils.h"

namespace Qbead
{
    /**
     * @brief Minimal interface for painting a horizontal LED band.
     *
     * Decouples ProbabilityDisplay from Qbead to avoid a circular dependency;
     * Qbead implements this interface.
     */
    struct BandPainter
    {
        /**
         * @brief Sets every LED in a horizontal band to one colour.
         * @param band      Band index, 0 to nsections (the two poles are single-pixel bands).
         * @param color     Packed colour 0x00RRGGBB.
         * @param inversion If true, band indices count from the opposite pole.
         */
        virtual void setBandColor(int band, uint32_t color, bool inversion = true) = 0;
    };

    /**
     * @brief Renders probabilities and counts as bands that fill up the sphere.
     *
     * Bands are painted through a BandPainter. Band 0 is the bottom (south pole) band
     * as seen by this class; the painter's default inversion handles the LED mapping.
     */
    class ProbabilityDisplay
    {
        BandPainter &painter; /**< Target that paints the bands (normally the Qbead). */

        // Colours
        const uint32_t SHOT_THR_COLOUR = color(0, 0, 255);   /**< Pulsing threshold-band colour. #2a4bd1 */
        const uint32_t FULL_BAND_COLOUR = color(0, 255, 0);  /**< Colour of a completely filled band. TODO REdo into proper colour maps also with HSV */
        const uint32_t EMPTY_BAND_COLOUR = color(0, 0, 255); /**< Colour of an empty band in fillColorMap(). Also so I have 4 different onces */

        const uint8_t nbands; /**< Number of bands to paint. */

        uint32_t currentFillColour = 0; /**< Most recent colour produced by fillColorMap(). */

        // uint32_t fadeStartTime = 0;
        const uint32_t fadeInterval = 2000; /**< Period of the threshold-band pulse, in ms. */

    public:
        /**
         * @brief Constructs a display bound to a band painter.
         * @param bp     Painter used to set band colours; must outlive this object.
         * @param nbands Number of horizontal bands on the device.
         * @todo Probably add a setupProbMode() to Qbead?
         */
        ProbabilityDisplay(BandPainter &bp, const uint16_t nbands);

        /**
         * @brief Cross-fades from one colour to another via black.
         *
         * For phase < 0.5 the top colour fades out; for phase > 0.5 the bottom colour
         * fades in. At phase 0.5 the result is black.
         *
         * @param phase  Fade phase, 0-1.
         * @param topCol Colour being painted over the base.
         * @param botCol Base colour being painted over.
         * @return Blended colour.
         */
        uint32_t colourCrossFade(float phase, uint32_t topCol, uint32_t botCol);

        /**
         * @brief Area fraction of the sphere below the lower edge of a band.
         * @param b         Band index (south pole = band 0).
         * @param nsections Number of theta sections.
         * @return Area fraction, 0-1.
         */
        float bandEdgeArea(uint8_t b, uint8_t nsections);

        /**
         * @brief Fill level of a band when a fraction of the sphere's area is filled from the south.
         * @param p         Filled area fraction, 0-1.
         * @param b         Band index (south pole = band 0).
         * @param nsections Number of theta sections.
         * @return Band level, 0 (empty) to 1 (full).
         */
        float bandLevel(float p, uint8_t b, uint8_t nsections);

        /**
         * @brief Maps a band fill level to a colour and stores it in currentFillColour.
         * @param t Band fill level: 0 = EMPTY_BAND_COLOUR, 0.5 = black, 1 = FULL_BAND_COLOUR. Clamped to [0, 1].
         * @return The mapped colour.
         */
        uint32_t fillColorMap(float t);

        /**
         * @brief Fills bands from the bottom in proportion to count / total.
         *
         * Fully covered bands get FULL_BAND_COLOUR; the single partially filled band
         * gets a colour from fillColorMap(). Bands above it are left untouched.
         *
         * @param count Number of hits.
         * @param total Total number of shots; must be non-zero and >= @p count.
         * @note Invalid arguments print an error to Serial and paint nothing.
         */
        void showSingleLiquidFill(uint32_t count, uint32_t total);

        /**
         * @brief Fills bands from both poles: successes rise from the bottom, failures fall from the top.
         *
         * Each band's colour is the gamma-corrected sum of the scaled success and
         * failure colours (linear band mapping).
         *
         * @param botCount     Count filled from the bottom (south pole).
         * @param topCount     Count filled from the top (north pole).
         * @param total        Total count; must be non-zero and >= botCount + topCount.
         * @param successColor Colour of the bottom fill.
         * @param failColor    Colour of the top fill.
         * @note Invalid arguments print an error to Serial and paint nothing.
         */
        void showDualLiquidFill(uint32_t botCount, uint32_t topCount, uint32_t total,
                                uint32_t successColor, uint32_t failColor);

        /**
         * @brief Pulses the threshold band between SHOT_THR_COLOUR and an underlying colour.
         *
         * Uses a cosine-shaped pulse with period fadeInterval. Currently always paints band 4.
         *
         * @param fadeStartTime Reference time of the pulse, in ms (millis()).
         * @param underColor    Colour shown when the pulse is dark.
         */
        void showThresholdBand(uint32_t fadeStartTime, uint32_t underColor);
    };
}
#endif // QBEAD_PROB_DISPLAY_H
