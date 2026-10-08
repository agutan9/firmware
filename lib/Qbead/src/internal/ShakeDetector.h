#include <math.h>

/**
 * @brief Detects a deliberate shake along one axis (currently the vertical, gravity axis).
 *
 * @par Vocabulary
 * - **Stroke**: one rapid physical movement in one direction.
 * - **Peak**: an extreme of the on-axis acceleration. For a smooth shake, peaks occur at
 *   the turnarounds between strokes (where the movement reverses) and zero crossings
 *   occur mid-stroke, at maximum speed.
 * - **Peak amplitude**: the size of one peak (positive peak > 0, negative peak < 0).
 * - **Peak-to-peak amplitude**: positive peak amplitude minus negative peak amplitude.
 * - **Half-period**: the time between a positive and the following negative peak.
 * - **Sphere frame**: the local coordinate basis of the Qbead device.
 *   Local meaning it stays fixed w.r.t the device, thus transforming under user rotations
 *   in the global coordinate basis; e.g. the Z-axis stays aligned with the LED-poles.
 * - **World frame**: the global coordinate basis of the Qbead device.
 *   Global meaning it stays fixed w.r.t. the user's world, thus staying fixed under
 *   user rotations; e.g. the Z-axis stays aligned with the gravity axis.
 *
 * @par Units and frames
 * All accelerations and amplitudes are in g. All final axis are unit vecors.
 * All vectors passed to and stored in this struct are expressed in the sphere frame.
 *
 * @par Method
 * Each sample has the gravity estimate subtracted, and the remainder is split into
 * an on-axis part (along shakeAxis) and an off-axis part (perpendicular to it).
 * A peak is registered when the on-axis part is large enough while the off-axis part
 * is small enough; i.e. the sampled shake is within a tolerance cone around the intended axis.
 * A shake is reported when a positive and a negative peak occur within
 * maxHalfPeriodMs of each other and their peak-to-peak amplitude is large enough.
 *
 * @par Usage
 * Call update() exactly once per sensor sample, with the unsmoothed acceleration;
 * smoothing such as the 100 ms exp-LPF in readIMU removes the fast strokes this
 * detector looks for. With externalGravity, gap handling is the gravity estimator's job;
 * the isInitialised flag here only matters for the internal gravity filter.
 * Without externalGravity, set isInitialised to false to recallibrate the gravityEstimate.
 */
struct ShakeDetector
{
    /** @name Detection behaviour tunable parameters
     *  @{ */

    /**
     * Number of alternating-sign peaks required before a shake is reported.
     * One stroke gives 2 (accelerate, brake), down-and-up gives 3, down-up-down gives 4.
     */
    uint8_t minAlternatingPeaks = 3;

    /**
     * Minimum amplitude each peak must reach, per side. The total swing is therefore
     * at least 2 x this value. Raise it to demand harder strokes.
     */
    float minPeakAmplitude = 0.5f;

    /**
     * Minimum peak-to-peak amplitude (positive peak minus negative peak).
     * It only has an effect when it exceeds 2 x minPeakAmplitude; it then allows
     * asymmetric shakes (e.g. +0.5 and -1.4) while still demanding a large total swing.
     */
    float minPeakToPeakAmplitude = 1.9f;

    /**
     * On-axis acceleration below which a sample counts as close to the shake signal's zero crossing.
     * The last such sample before the first peak supplies shakeAxisAtStart.
     * Keep it above the noise and gravity-offset floor, and well below minPeakAmplitude.
     */
    float nearZeroThreshold = 0.1f;

    /**
     * Maximum ratio of off-axis to on-axis acceleration for a peak to count.
     * It defines a cone around the shake axis with half-angle atan(ratio):
     * 0.45 is about 24 degrees. Smaller values demand a straighter shake.
     */
    float maxOffAxisRatio = 0.45f;

    /**
     * Maximum time between a positive and a negative peak (half a period of the shake).
     * 300 ms corresponds to a minimum shake frequency of roughly 1.7 Hz.
     */
    uint32_t maxHalfPeriodMs = 300;

    /** Minimum time between two reported shakes in ms. */
    uint32_t cooldownMs = 500;

    /**
     * How slowly the internal gravity estimate adapts in seconds; i.e. it is Tau,
     * the time constant of a first-order low-pass filter: about 63% of a step change
     * is absorbed in 1 Tau.
     * Larger values follow rotations more slowly but are less disturbed by shaking.
     * Not used when externalGravity is passed to update().
     */
    float intGravityEstTimeConstantSeconds = 1.0f;
    /** @} */

    /** @name Saved outputs
     *  @{ */

    /**
     * Unit shake axis (sphere frame) captured at the last stable shake axis before a detected shake.
     * Useful for for acting upon the intended sphere basis before the shake action potentially displaced it.
     * For example we use it as the intended measurement axis in the case of shake to measure.
     *
     * Important! Copy it immediately when update() returns true as subsequent update calls can overwrite it.
     */
    float shakeAxisAtStart[3] = {0.f, 0.f, 1.f};
    /** @} */

    /** @name Internal state
     *  @{ */
    /** Gravity (sphere frame), subtracted from every sample. */
    float gravityEstimate[3] = {0.f, 0.f, 1.f};
    /** Unit axis the detector listens along; re-derived every update. */
    float shakeAxis[3] = {0.f, 0.f, 1.f};
    /** Continuously updating shakeAxis around nearZero before a shake is initiated. Becomes shakeAxisAtStart if not stale.*/
    float shakeAxisPrePrimaryPeak[3] = {0.f, 0.f, 1.f};
    /** True once gravityEstimate holds valid data. Set to false to restart (e.g. after a timing gap). */
    bool isInitialised = false;
    /** Largest positive on-axis acceleration in the current window. */
    float positivePeakAmplitude = 0.f;
    /** Most negative on-axis acceleration in the current window (signed, <= 0). */
    float negativePeakAmplitude = 0.f;
    /** True while a positive peak is inside the pairing window. */
    bool hasPositivePeak = false;
    /** True while a negative peak is inside the pairing window. */
    bool hasNegativePeak = false;
    /** Time of the most recent qualifying positive sample. */
    uint32_t positivePeakTimeMs = 0;
    /** Time of the most recent qualifying negative sample. */
    uint32_t negativePeakTimeMs = 0;
    /** Time of the last reported shake (for the cooldown). */
    uint32_t lastShakeTimeMs = 0;
    /** Time of the last sample before a potential shake was initiated. */
    uint32_t lastNearZeroTimeMs = 0;
    /** Sign of the most recent peak (+1 or -1); 0 if none yet. */
    int8_t lastPeakSign = 0;
    /** Number of sign changes in the current attempt, counting the first peak. */
    uint8_t alternatingPeakCount = 0;
    /** @} */

    /**
     * @brief Processes one accelerometer sample.
     *
     * @param accelSphere      Acceleration in the sphere frame (3 elements), in g.
     *                         Unfiltered acceleration data works best as strokes rely on high freqs.
     * @param dtSeconds        Time since the previous sample, in seconds.
     * @param nowMillis        Current time in ms (e.g. millis()).
     *                         Unsigned arithmetic keeps the time windows correct across wraparound.
     * @param[in] externalGravity  Optional gravity vector in the sphere frame (3 elements).
     *                             If nullptr, gravity is filtered internally from accelSphere.
     * @return True exactly once per detected shake.
     */
    bool update(const float accelSphere[3], float dtSeconds, uint32_t nowMillis,
                const float *externalGravity = nullptr)
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
            hasPositivePeak = hasNegativePeak = false;
            alternatingPeakCount = 0;
            lastPeakSign = 0;
            lastShakeTimeMs = nowMillis;
            return true;
        }
        return false;
    }
};