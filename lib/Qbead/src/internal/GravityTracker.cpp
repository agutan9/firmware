/**
 * @file GravityTracker.cpp
 * @brief Implementation of Qbead::GravityTracker.
 */
#include "GravityTracker.h"

namespace Qbead
{
    void GravityTracker::update(const float accelSphere[3], const float gyroSphereDps[3], float dtSeconds)
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
}
