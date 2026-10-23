/**
 * @file ProbabilityDisplay.cpp
 * @brief Implementation of Qbead::ProbabilityDisplay.
 */
#include "ProbabilityDisplay.h"

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

namespace Qbead
{
    // TODO: Probably add a setupProbMode() to Qbead?
    ProbabilityDisplay::ProbabilityDisplay(BandPainter &bp, const uint16_t nbands) : painter(bp), nbands(nbands) {}

    // Add to Utils probably
    // Maybe have it not be black in the middle but a combination?
    // Ask AI for the refactor
    // Or just delete as I might not end up using it
    // u: 0..1 fade phase. topCol = the colour we are painting over the base, botCol = the base colour being painted over
    uint32_t ProbabilityDisplay::colourCrossFade(float phase, uint32_t topCol, uint32_t botCol)
    {
        float a = (phase < 0.5f) ? (1.f - 2.f * phase) : 0.f; // threshold fades out
        float b = (phase > 0.5f) ? (2.f * phase - 1.f) : 0.f; // underlying fades in
        return addColor(scaleColor(a, topCol), scaleColor(b, botCol));
    }

    // area fraction of the sphere below the lower edge of band b (south pole = band 0)
    float ProbabilityDisplay::bandEdgeArea(uint8_t b, uint8_t nsections)
    {
        float theta = constrain((b - 0.5f) * 180.f / nsections, 0.f, 180.f);
        return (1.f - cos_deg(theta)) * 0.5f;
    }

    // 0..1 level of band b when a fraction p of the sphere's area is filled from the south
    float ProbabilityDisplay::bandLevel(float p, uint8_t b, uint8_t nsections)
    {
        float lo = bandEdgeArea(b, nsections);
        float hi = bandEdgeArea(b + 1, nsections); // clamps to 1.0 past the north pole
        return constrain((p - lo) / (hi - lo), 0.f, 1.f);
    }

    // t = band fill level: 0 = red (empty), 0.5 = black, 1 = green (full)
    uint32_t ProbabilityDisplay::fillColorMap(float t)
    {
        t = constrain(t, 0.f, 1.f); // TODO?
        if (t < 0.5f)
        {
            currentFillColour = scaleColor(1.f - 2.f * t, EMPTY_BAND_COLOUR);
        }
        else
        {
            currentFillColour = scaleColor(2.f * t - 1.f, FULL_BAND_COLOUR);
        }
        return currentFillColour;
    }

    // TODO: Better handle color setting
    void ProbabilityDisplay::showSingleLiquidFill(uint32_t count, uint32_t total)
    {
        if (total == 0 || total < count)
        {
            Serial.printf("[ERROR]{PROB.DISPLAY} Invalid count (%lu) and total (%lu) args passed to showSingleLiquidfFill!\n",
                          (ulong)count, (ulong)total);
            return;
        }
        float fillRatio = ((float)count / total) * nbands;
        uint8_t fullBands = (uint8_t)fillRatio; // truncate down
        // no colour map needed for all the already filled bands
        for (uint8_t fb_i = 0; fb_i < fullBands; fb_i++)
        {
            painter.setBandColor(fb_i, FULL_BAND_COLOUR);
        }
        // Skip if all bands are full or no fractional remainder
        float fracBand = fillRatio - fullBands;
        if (fullBands < nbands && fracBand > 1e-5)
        {
            // Only one band at a time can have fill-up gradient
            painter.setBandColor(fullBands, fillColorMap(fracBand));
        }
    }

    void ProbabilityDisplay::showDualLiquidFill(uint32_t botCount, uint32_t topCount, uint32_t total,
                                                uint32_t successColor, uint32_t failColor)
    {
        if (total == 0 || (uint64_t)botCount + topCount > total)
        {
            Serial.printf("[ERROR]{PROB.DISPLAY} Invalid botCount (%lu), topCount (%lu) and total (%lu) args passed to showDualLiquidfFill!\n",
                          (ulong)botCount, (ulong)topCount, (ulong)total);
            return;
        }
        // TODO: My way? Or are these constraints fine?
        // Re-use bottom-up
        // showSingleLiquidFill(bead, countBot, total);
        //// New top-down second fill
        // float fillRatio = ((float)countTop / total) * nbands;
        // uint8_t fullBands = (uint8_t)fillRatio; // truncate down
        //// no colour map needed for all the already filled bands
        // for (uint8_t fb_i = 1; fb_i <= fullBands; fb_i++)
        //{
        //     setBandColour(bead, nbands - fb_i, FULL_BAND_COLOUR);
        // }
        //// Skip if all bands are full
        // if (fullBands < nbands)
        //{
        //     // Only one band at a time can have fill-up gradient
        //     setBandColour(bead, nbands - fullBands - 1, fillColorMap(fillRatio - fullBands));
        // }

        // LINEAR
        float win = (float)botCount / total * nbands;  // rises from the south pole
        float loss = (float)topCount / total * nbands; // falls from the north pole

        // AREA
        // float winP = (float)botCount / total;
        // float lossP = (float)topCount / total;

        for (uint8_t b = 0; b < nbands; b++)
        {

            // LINEAR
            float w = constrain(win - b, 0.f, 1.f);
            float l = constrain(loss - (nbands - 1 - b), 0.f, 1.f);

            // AREA MAPPED
            // float w = bandLevel(winP,  b,             QB_NSECTIONS);
            // float l = bandLevel(lossP, QB_NSECTIONS - b, QB_NSECTIONS);

            // NO GAMMA
            // painter.setBandColor(b, addColor(scaleColor(w, color(0, 255, 0)),
            //                                 scaleColor(l, color(255, 0, 0))));

            // GAMMA
            painter.setBandColor(b, Adafruit_NeoPixel::gamma32(
                                        addColor(scaleColor(w, successColor), scaleColor(l, failColor))));
        }
    }

    void ProbabilityDisplay::showThresholdBand(uint32_t fadeStartTime, uint32_t underColor)
    {
        uint32_t fadeNow = millis() - fadeStartTime % fadeInterval;
        // fadeStartTime
        uint8_t phase = (uint8_t)((fadeNow * 255UL) / fadeInterval);
        // Want to start at 1.0 (sin8 starts at 0.5) so need to phase shift to a cosine
        float luxLevel = Adafruit_NeoPixel::sine8(phase + 64) / 255.0f;
        // Set the 5th band: 0.75 * 7 = 5.25 -> 5
        // TODO: Ensure that the supposed underlying colour gets shown isntead of overwritten with black
        // painter.setBandColor(THRESHOLD_BAND, scaleColor(luxLevel, SHOT_THR_COLOUR));
        // painter.setBandColor((uint8_t)(0.75f * nbands),
        painter.setBandColor(4,
                             addColor(scaleColor(luxLevel, SHOT_THR_COLOUR),
                                      scaleColor(1.f - luxLevel, underColor)));
    }
}
