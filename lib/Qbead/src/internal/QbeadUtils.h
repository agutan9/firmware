#ifndef QBEAD_UTILS_H
#define QBEAD_UTILS_H

#include <math.h>
#include <Arduino.h>

namespace Qbead
{
// default configs
#define QB_LEDPIN 10
#define QB_PIXELCONFIG NEO_GRB + NEO_KHZ800
#define QB_NSECTIONS 6
#define QB_NLEGS 12
#define QB_IMU_ADDR 0x6A
#define QB_IX 2
#define QB_IY 0
#define QB_IZ 1
#define QB_SX 0
#define QB_SY 1
#define QB_SZ 1
#define INNER_STATE_COUNT 6
#define CYCLING_TIME 600

// LSM6DS3 filter settings
#define LSM6DS3_ACC_GYRO_LPF2_XL_EN 0x80
#define LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_50 0x00
#define LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_100 0x20
#define LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_9 0x40
#define LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_400 0x60
    static constexpr uint32_t CYCLING_TIME = 600;
    static constexpr uint32_t ENTANGLE_WINDOW_MS = 100;

#define QB_MAX_PRPH_CONNECTION 2

    const uint8_t QB_UUID_SERVICE[] =
        {0x45, 0x8d, 0x08, 0xaa, 0xd6, 0x63, 0x44, 0x25, 0xbe, 0x12, 0x9c, 0x35, 0xc6, 0x1f, 0x0c, 0xe3};
    const uint8_t QB_UUID_COL_CHAR[] =
        {0x45, 0x8d, 0x08, 0xaa, 0xd6, 0x63, 0x44, 0x25, 0xbe, 0x12, 0x9c, 0x35, 0xc6 + 1, 0x1f, 0x0c, 0xe3};
    const uint8_t QB_UUID_SPH_CHAR[] =
        {0x45, 0x8d, 0x08, 0xaa, 0xd6, 0x63, 0x44, 0x25, 0xbe, 0x12, 0x9c, 0x35, 0xc6 + 2, 0x1f, 0x0c, 0xe3};
    const uint8_t QB_UUID_ACC_CHAR[] =
        {0x45, 0x8d, 0x08, 0xaa, 0xd6, 0x63, 0x44, 0x25, 0xbe, 0x12, 0x9c, 0x35, 0xc6 + 3, 0x1f, 0x0c, 0xe3};
    const uint8_t QB_UUID_TAP_CHAR[] =
        {0x45, 0x8d, 0x08, 0xaa, 0xd6, 0x63, 0x44, 0x25, 0xbe, 0x12, 0x9c, 0x35, 0xc6 + 4, 0x1f, 0x0c, 0xe3};
    const uint8_t QB_UUID_DATA_CHAR[] =
        {0x45, 0x8d, 0x08, 0xaa, 0xd6, 0x63, 0x44, 0x25, 0xbe, 0x12, 0x9c, 0x35, 0xc6 + 5, 0x1f, 0x0c, 0xe3};

    const uint8_t zerobuffer20[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

    // TODO manage namespaces better
    static uint32_t color(uint8_t r, uint8_t g, uint8_t b)
    {
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }

    static uint8_t redch(uint32_t rgb)
    {
        return rgb >> 16;
    }

    static uint8_t greench(uint32_t rgb)
    {
        return (0x00ff00 & rgb) >> 8;
    }

    static uint8_t bluech(uint32_t rgb)
    {
        return 0x0000ff & rgb;
    }

    static uint32_t addColor(uint32_t c0, uint32_t c1)
    {
        uint8_t r = min(0xff, (int)redch(c0) + redch(c1));
        uint8_t g = min(0xff, (int)greench(c0) + greench(c1));
        uint8_t b = min(0xff, (int)bluech(c0) + bluech(c1));
        return color(r, g, b);
    }

    static uint32_t scaleColor(float a, uint32_t c)
    {
        uint8_t r = min(0xff, a * redch(c));
        uint8_t g = min(0xff, a * greench(c));
        uint8_t b = min(0xff, a * bluech(c));
        return color(r, g, b);
    }

    static uint32_t scaleColorQuad(float a, uint32_t c)
    {
        float a2 = a * a;
        uint8_t r = min(0xff, a2 * redch(c));
        uint8_t g = min(0xff, a2 * greench(c));
        uint8_t b = min(0xff, a2 * bluech(c));
        return color(r, g, b);
    }

    static uint32_t scaleColor_8bit(uint8_t a, uint32_t c)
    {
        uint8_t r = min(0xff, (int)a * redch(c) / 255);
        uint8_t g = min(0xff, (int)a * greench(c) / 255);
        uint8_t b = min(0xff, (int)a * bluech(c) / 255);
        return color(r, g, b);
    }

    uint32_t colorWheel(uint8_t wheelPos)
    {
        wheelPos = 255 - wheelPos;
        if (wheelPos < 85)
        {
            return color(255 - wheelPos * 3, 0, wheelPos * 3);
        }
        if (wheelPos < 170)
        {
            wheelPos -= 85;
            return color(0, wheelPos * 3, 255 - wheelPos * 3);
        }
        wheelPos -= 170;
        return color(wheelPos * 3, 255 - wheelPos * 3, 0);
    }

    // #### Parabolic wave
    // Similarly to the triangular wave, this function is useful for periodically
    // pulsating patterns. However, the profile of this function resembles a beating
    // heart more closely and it can provide for more pleasing visuals.
    // ![Depiction of the parabolic wave.](./parabola_wave.png)
    uint8_t parabolaWave(uint8_t x)
    {
        uint8_t xm = x;
        if (xm > 0x7f)
        {
            xm = 0xff - xm;
        }
        return (xm * xm) >> 6;
    }

    uint32_t colorWheel_deg(float wheelPos)
    {
        return colorWheel(wheelPos * 255 / 360);
    }

    float sign(float x)
    {
        if (x > 0)
            return +1;
        else
            return -1;
    }

    // z = cos(t)
    // x = cos(p)sin(t)
    // y = sin(p)sin(t)
    // Return the angle in radians between the x-axis and the line to the point (x, y)
    float phi(float x, float y)
    {
        return atan2(y, x);
    }
    float phi(float x, float y, float z)
    {
        return phi(x, y);
    }
    float theta(float x, float y, float z)
    {
        float ll = x * x + y * y + z * z;
        float l = sqrt(ll);
        float theta = acos(z / l);
        return theta;
    }

    static float toRadians(const float angle)
    {
        return angle / 180 * M_PI;
    }

    static float toDegrees(const float angle)
    {
        return angle * 180 / M_PI;
    }

    static float sin_deg(const float angle)
    {
        return sin(toRadians(angle));
    }

    static float cos_deg(const float angle)
    {
        return cos(toRadians(angle));
    }

    // Mod function that only returns positive numbers
    static float modulo(const float a, const float b)
    {
        return fmod(a, b) + (a < 0) * b;
    }

    bool checkThetaAndPhi(float theta, float phi)
    {
        return theta >= 0 && theta <= 180 && phi >= 0 && phi <= 360;
    }

    int computePixelIndex(int leg, int pixel)
    {
        leg = QB_NLEGS - leg; // invert direction for the phi angle, because the PCB is set up as a left-handed coordinate system
        leg = leg % QB_NLEGS;
        if (leg == 0)
        {
            return pixel;
        }
        else if (pixel == 0 || pixel == QB_NSECTIONS)
        {
            return pixel;
        }
        return (QB_NSECTIONS + 1) + (leg - 1) * (QB_NSECTIONS - 1) + pixel - 1;
    }

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
}

#endif