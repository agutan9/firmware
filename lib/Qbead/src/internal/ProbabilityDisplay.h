#ifndef QBEAD_PROB_DISPLAY_H
#define QBEAD_PROB_DISPLAY_H


namespace Qbead
{
    // need an interface to decouple circular qbead dependance
    struct BandPainter { 
        virtual void setBandColor(int band, uint32_t color) = 0;
    };

    class ProbabilityDisplay
    {
        BandPainter &painter;

        // Colours
        const uint32_t SHOT_THR_COLOUR = color(0, 0, 255);   // #2a4bd1
        const uint32_t FULL_BAND_COLOUR = color(0, 255, 0);  // TODO REdo into proper colour maps also with HSV
        const uint32_t EMPTY_BAND_COLOUR = color(0, 0, 255); // Also so I have 4 different onces

        const uint8_t nbands;

        uint32_t currentFillColour = 0;

        //uint32_t fadeStartTime = 0;
        const uint32_t fadeInterval = 2000; 


    public:
        // TODO: Probably add a setupProbMode() to Qbead?
        ProbabilityDisplay(BandPainter &bp, const uint16_t nbands) : painter(bp), nbands(nbands) {}

        // Add to Utils probably
        // Maybe have it not be black in the middle but a combination?
        // Ask AI for the refactor
        // Or just delete as I might not end up using it
        // u: 0..1 fade phase. topCol = the colour we are painting over the base, botCol = the base colour being painted over
        uint32_t colourCrossFade(float phase, uint32_t topCol, uint32_t botCol)
        {
            float a = (phase < 0.5f) ? (1.f - 2.f * phase) : 0.f; // threshold fades out
            float b = (phase > 0.5f) ? (2.f * phase - 1.f) : 0.f; // underlying fades in
            return addColor(scaleColor(a, topCol), scaleColor(b, botCol));
        }

        // t = band fill level: 0 = red (empty), 0.5 = black, 1 = green (full)
        uint32_t fillColorMap(float t)
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

        void showSingleLiquidFill(uint32_t count, uint32_t total)
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

        void showDualLiquidFill(uint32_t botCount, uint32_t topCount, uint32_t total)
        {
            if (total == 0 || (uint64_t)botCount + topCount > total)
            {
                Serial.printf("[ERROR]{PROB.DISPLAY} Invalid botCount (%lu), topCount (%lu) and total (%lu) args passed to showDualLiquidfFill!\n",
                              (ulong)botCount, (ulong)topCount, (ulong)total);
                return;
            }
            // TODO: My way? Or are these constraints fine?
            // Re-use bottom-up
            //showSingleLiquidFill(bead, countBot, total);
            //// New top-down second fill
            //float fillRatio = ((float)countTop / total) * nbands;
            //uint8_t fullBands = (uint8_t)fillRatio; // truncate down
            //// no colour map needed for all the already filled bands
            //for (uint8_t fb_i = 1; fb_i <= fullBands; fb_i++)
            //{
            //    setBandColour(bead, nbands - fb_i, FULL_BAND_COLOUR);
            //}
            //// Skip if all bands are full
            //if (fullBands < nbands)
            //{
            //    // Only one band at a time can have fill-up gradient
            //    setBandColour(bead, nbands - fullBands - 1, fillColorMap(fillRatio - fullBands));
            //}
            float win  = (float)botCount  / total * nbands;   // rises from the south pole
            float loss = (float)topCount / total * nbands;   // falls from the north pole

            for (uint8_t b = 0; b < nbands; b++) {
                float w = constrain(win  - b,                0.f, 1.f);
                float l = constrain(loss - (nbands - 1 - b), 0.f, 1.f);
                painter.setBandColor(b, addColor(scaleColor(w, color(0, 255, 0)),
                                                scaleColor(l, color(255, 0, 0))));
            }
        }

        void showThresholdBand(uint32_t fadeStartTime, uint32_t underColor)
        {
            uint32_t fadeNow = millis() - fadeStartTime % fadeInterval;
            //fadeStartTime 
            uint8_t phase = (uint8_t)((fadeNow * 255UL) / fadeInterval);
            // Want to start at 1.0 (sin8 starts at 0.5) so need to phase shift to a cosine
            float luxLevel = Adafruit_NeoPixel::sine8(phase + 64) / 255.0f;
            // Set the 5th band: 0.75 * 7 = 5.25 -> 5
            // TODO: Ensure that the supposed underlying colour gets shown isntead of overwritten with black
            //painter.setBandColor(THRESHOLD_BAND, scaleColor(luxLevel, SHOT_THR_COLOUR));
            painter.setBandColor(4,
                addColor(scaleColor(luxLevel, SHOT_THR_COLOUR),
                 scaleColor(1.f - luxLevel, underColor)));
        }
    };
}
#endif