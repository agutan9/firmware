/**
 * @file ShakeDetector.h
 * @brief Software detector for deliberate vertical shake gestures.
 *
 * Implementation lives in ShakeDetector.cpp.
 */
#ifndef QBEAD_SHAKE_DETECT_H
#define QBEAD_SHAKE_DETECT_H

#include <math.h>
#include <stdint.h>

namespace Qbead
{
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
                    const float *externalGravity = nullptr);
    };
}
#endif // QBEAD_SHAKE_DETECT_H
