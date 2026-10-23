/**
 * @file ShakeDetector.cpp
 * @brief Implementation of Qbead::ShakeDetector.
 */
#include "ShakeDetector.h"

#include <Arduino.h>
#include <string.h>

namespace Qbead
{
    bool ShakeDetector::update(const float accelSphere[3], float dtSeconds, uint32_t nowMillis,
                               const float *externalGravity)
    {
        // 1. Gravity reference: use the external estimate, or seed/filter our own.
        if (externalGravity)
        {
            memcpy(gravityEstimate, externalGravity, sizeof gravityEstimate);
            isInitialised = true;
        }
        else if (!isInitialised)
        {
            memcpy(gravityEstimate, accelSphere, sizeof gravityEstimate);
            isInitialised = true;
        }

        // 2. Shake axis. Currently this is the gravity direction (vertical shakes).
        //    FUTURE: replace this block to select another axis (e.g. world X or Y).
        //    TODO PR: This can be extended to track world X or Y.
        //    However, these are not as constant as world Z and would need some further work
        //    1. If you rotate the way you are interfacing with the device the world X or Y
        //       would not notice. If you turn 90deg your left-right (X) is now mapped to
        //       what was front-back (Y)
        //    2. Hence we would likely need a callibration step. Not impossible to implement,
        //       but also not trivial
        //    3. Any-shake in the X-Y plane would get around this but also requires some
        //       non-trivial extension to current code
        //    4. Without a magnometer the axis might drift over-time (would need further looking into)
        float gravityNorm = sqrtf(gravityEstimate[0] * gravityEstimate[0] +
                                  gravityEstimate[1] * gravityEstimate[1] +
                                  gravityEstimate[2] * gravityEstimate[2]);
        // guard against a zero vector (ex. bad I2C read, free-fall..)
        if (gravityNorm < 1e-6f)
        {
            isInitialised = false; // re-seed on next sample
            return false;
        }
        // update the gravity unit vector
        for (int i = 0; i < 3; i++)
            shakeAxis[i] = gravityEstimate[i] / gravityNorm;

        // 3. Remove gravity, then split into on-axis and off-axis parts.
        //    At rest the accelerometer reads 1 g "up"; subtracting the gravity estimate leaves ~0 (i.e. only the user-induced acceleration).
        float linearAccel[3] = {accelSphere[0] - gravityEstimate[0],
                                accelSphere[1] - gravityEstimate[1],
                                accelSphere[2] - gravityEstimate[2]};
        // projection onto detection axis; requires shakeAxis to be unit vector. Retains sign.
        float onAxisAccelAmp = linearAccel[0] * shakeAxis[0] + linearAccel[1] * shakeAxis[1] + linearAccel[2] * shakeAxis[2];
        // removing parallel part leaves us with the perpendicular component as a vector
        float offAxisVector[3] = {linearAccel[0] - onAxisAccelAmp * shakeAxis[0],
                                  linearAccel[1] - onAxisAccelAmp * shakeAxis[1],
                                  linearAccel[2] - onAxisAccelAmp * shakeAxis[2]};
        // We only need the off-axis length for the cone test so take the norm
        float offAxisAccelAmp = sqrtf(offAxisVector[0] * offAxisVector[0] +
                                      offAxisVector[1] * offAxisVector[1] +
                                      offAxisVector[2] * offAxisVector[2]);

        // 4. Internal gravity filter (only without externalGravity).
        //    Less accurate than gravityTracker but doesn't require gyro (higher power consumption).
        if (!externalGravity)
        {
            float filterGain = dtSeconds / ((hasPositivePeak || hasNegativePeak)
                                                ? 4 * intGravityEstTimeConstantSeconds // slow down mid-shake to preserve gravity estimate
                                                : intGravityEstTimeConstantSeconds);
            // never extrapolate past the measurement after a long gap
            filterGain = fminf(filterGain, 1.0f);
            for (int i = 0; i < 3; i++)
                gravityEstimate[i] += filterGain * (accelSphere[i] - gravityEstimate[i]);
        }

        // 5. Forget peaks older than the pairing window.
        if (hasPositivePeak && nowMillis - positivePeakTimeMs > maxHalfPeriodMs)
            hasPositivePeak = false;
        if (hasNegativePeak && nowMillis - negativePeakTimeMs > maxHalfPeriodMs)
            hasNegativePeak = false;

        // 6. Register a qualifying sample: large enough and inside the cone around the axis.
        bool awaitingFirstPeak = !(hasPositivePeak || hasNegativePeak);
        float onAxisAbsAmp = fabsf(onAxisAccelAmp);
        // continuously track stable shakeAxis estimates before any shake is initiated.
        if (awaitingFirstPeak && onAxisAbsAmp < nearZeroThreshold)
        {
            memcpy(shakeAxisPrePrimaryPeak, shakeAxis, sizeof shakeAxis);
            lastNearZeroTimeMs = nowMillis;
        }
        // logic for during an initiated shake attempt.
        if (onAxisAbsAmp > minPeakAmplitude && offAxisAccelAmp < maxOffAxisRatio * onAxisAbsAmp)
        {
            int8_t peakSign = (onAxisAccelAmp > 0) ? 1 : -1;
            // Stroke ramping up towards first valid peak
            if (awaitingFirstPeak)
            {
                // safety check for staleness, if failed fall back on current live shakeAxis.
                // current maxHalfPeriodMs is generous and mostly targets repeated too slow attempts preceding a fast one.
                bool nearZeroIsRecent = nowMillis - lastNearZeroTimeMs <= maxHalfPeriodMs;
                memcpy(shakeAxisAtStart, nearZeroIsRecent ? shakeAxisPrePrimaryPeak : shakeAxis, sizeof shakeAxis);
                // init/reset peak counting
                alternatingPeakCount = 0;
                lastPeakSign = 0;
            }
            if (peakSign != lastPeakSign)
            {
                // Don't double count the same (increasing) peak
                alternatingPeakCount++;
                lastPeakSign = peakSign;
            }
            // Post first peak we detect until desired peak count is reached
            if (onAxisAccelAmp > 0)
            {
                if (!hasPositivePeak || onAxisAccelAmp > positivePeakAmplitude)
                    positivePeakAmplitude = onAxisAccelAmp;
                positivePeakTimeMs = nowMillis;
                hasPositivePeak = true;
            }
            else
            {
                if (!hasNegativePeak || onAxisAccelAmp < negativePeakAmplitude)
                    negativePeakAmplitude = onAxisAccelAmp;
                negativePeakTimeMs = nowMillis;
                hasNegativePeak = true;
            }
        }

        // 7. Report a shake: Both peaks present, large enough swing, cooldown over.
        float peakToPeakAmplitude = positivePeakAmplitude - negativePeakAmplitude;
        if (hasPositivePeak && hasNegativePeak &&
            alternatingPeakCount >= minAlternatingPeaks &&
            peakToPeakAmplitude > minPeakToPeakAmplitude &&
            nowMillis - lastShakeTimeMs > cooldownMs)
        {
            Serial.println("[INFO]{SHAKE_DETECT} Shake gesture was detected!");
            hasPositivePeak = hasNegativePeak = false;
            alternatingPeakCount = 0;
            lastPeakSign = 0;
            lastShakeTimeMs = nowMillis;
            return true;
        }
        return false;
    }
}
