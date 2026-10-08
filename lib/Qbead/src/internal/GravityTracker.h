#include <math.h>
#include <Arduino.h>

/**
 * @brief Tracks the gravity direction by fusing gyroscope and accelerometer readings.
 *
 * Because the accelerometer measures gravity and the device's motion simultaneously,
 * it can't seperate them risking pollution of gravity direction estimates by device motion.
 * Meanwhile the gyro measures instantaneous device rotation accurately, but integrated over time
 * it starts to drift. Hence, for a more accurate estimate we employ both at their strengths while
 * covering their weaknesses with each other.
 *
 * @par Output
 * gravityEstimate is the world frame "up" direction expressed in sphere frame coordinates,
 * as a unit vector. It points opposite to the gravitational acceleration, because that is what
 * a resting accelerometer measures. As the sphere rotates, its coordinates change.
 *
 * @par Method
 * Each step has two stages. **Predict**: rotate the estimate with the gyro (accurate during
 * motion). **Correct**: pull it slowly toward the accelerometer (removes drift). The pull is
 * weighted by how close |accel| is to 1 g, so shakes and impacts barely disturb it and thus
 * during such times the estimate coasts on the gyro. After returning to rest the built up
 * gyro drift is cancelled out over a few seconds.
 *
 * @par Units and frames
 * Accelerations in g, angular rates in degrees/s, times in seconds. All vectors are in the
 * sphere frame. The chip-to-sphere axis mapping must be a proper rotation (determinant +1),
 * as it is for the current ix/iy/iz and sx/sy/sz.
 *
 * @par Usage
 * Call update() once per sensor sample with the unsmoothed acceleration and the
 * bias-corrected gyro rate. Only use gravityEstimate when isInitialised is true.
 * A long gap in sampling makes the estimate invalid until a trustworthy sample re-seeds it.
 */
struct GravityTracker
{
    /** @name Tracking behaviour tunable parameters
     *  @{ */

    /**
     * The time constant (Tau) of a first-order low-pass filter: about 63% of a step change
     * is absorbed in 1 Tau.
     * How slowly to introduce accelerometer correction. Larger trusts the gyro for longer.
     */
    float accelCorrectionTimeConstantSeconds = 1.5f;

    /**
     * Accelerometer trust band, in g. The correction weight falls linearly to 0 as |accel|
     * deviates from 1 g by this amount, so shakes and impacts are ignored.
     */
    float accelTrustBand = 0.3f;

    /**
     * Longest time step (s) that is integrated. A longer gap invalidates the estimate
     * (isInitialised becomes false) instead of integrating a stale gyro rate.
     */
    float maxGapSeconds = 0.1f;

    /** The estimate is (re)seeded only from a sample whose |accel| is within this many g of 1 g. */
    float seedTrustBand = 0.1f;
    /** @} */

    /** @name State
     *  @{ */
    /** Unit "up" direction in the sphere frame. Valid only when isInitialised. */
    float gravityEstimate[3] = {0.f, 0.f, 1.f};
    /** False at start and after a gap, until a trustworthy sample re-seeds the estimate. */
    bool isInitialised = false;
    /** @} */

    /**
     * @brief Advances the estimate by one sample.
     *
     * @param accelSphere   Acceleration in the sphere frame (3 elements), in g. Unsmoothed.
     * @param gyroSphereDps Bias-corrected angular rate in the sphere frame (3 elements), in degrees/s.
     * @param dtSeconds     Time since the previous sample, in seconds.
     */
    void update(const float accelSphere[3], const float gyroSphereDps[3], float dtSeconds)
    {
        float accelMagnitude = sqrtf(accelSphere[0] * accelSphere[0] +
                                     accelSphere[1] * accelSphere[1] +
                                     accelSphere[2] * accelSphere[2]);
        float accelMagnitudeError = fabsf(accelMagnitude - 1.f); // deviation from 1 g

        // 1. A gap in sampling makes the integrated history unreliable.
        if (dtSeconds > maxGapSeconds)
            isInitialised = false;

        // 2. (Re)seed from the accelerometer, but only when it is trustworthy.
        if (!isInitialised)
        {
            if (accelMagnitudeError < seedTrustBand)
            {
                for (int i = 0; i < 3; i++)
                    gravityEstimate[i] = accelSphere[i] / accelMagnitude;
                isInitialised = true;
            }
            return; // otherwise keep the stale estimate and wait
        }

        // 3. Predict: in the sphere frame a world-fixed vector rotates as -omega x gravity.
        float omegaRadPerSecond[3] = {gyroSphereDps[0] * DEG_TO_RAD,
                                      gyroSphereDps[1] * DEG_TO_RAD,
                                      gyroSphereDps[2] * DEG_TO_RAD};
        float crossProduct[3] = {
            omegaRadPerSecond[1] * gravityEstimate[2] - omegaRadPerSecond[2] * gravityEstimate[1],
            omegaRadPerSecond[2] * gravityEstimate[0] - omegaRadPerSecond[0] * gravityEstimate[2],
            omegaRadPerSecond[0] * gravityEstimate[1] - omegaRadPerSecond[1] * gravityEstimate[0]};
        for (int i = 0; i < 3; i++)
            gravityEstimate[i] -= crossProduct[i] * dtSeconds;

        // 4. Correct: pull toward the accelerometer, weighted by how much it can be trusted.
        float accelTrust = fmaxf(0.f, 1.f - accelMagnitudeError / accelTrustBand); // 1 at 1 g, 0 beyond the band
        float correctionGain = (dtSeconds / accelCorrectionTimeConstantSeconds) * accelTrust;
        for (int i = 0; i < 3; i++)
            gravityEstimate[i] += correctionGain * (accelSphere[i] - gravityEstimate[i]);

        // 5. Renormalise to unit length (guarded against a zero vector).
        float norm = sqrtf(gravityEstimate[0] * gravityEstimate[0] +
                           gravityEstimate[1] * gravityEstimate[1] +
                           gravityEstimate[2] * gravityEstimate[2]);
        if (norm > 1e-6f)
            for (int i = 0; i < 3; i++)
                gravityEstimate[i] /= norm;
    }
};