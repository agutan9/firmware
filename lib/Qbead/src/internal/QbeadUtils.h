/**
 * @file QbeadUtils.h
 * @brief Shared configuration constants, colour helpers, angle maths, axis helpers and RNG for the Qbead.
 *
 * Implementations live in QbeadUtils.cpp. Configuration macros, BLE UUIDs and other
 * compile-time constants remain in this header.
 */
#ifndef QBEAD_UTILS_H
#define QBEAD_UTILS_H

#include <math.h>
#include <Arduino.h>

namespace Qbead
{
/** @name Default hardware configuration
 *  @{ */
#define QB_LEDPIN 10                        /**< Data pin of the first NeoPixel strip. */
#define QB_PIXELCONFIG NEO_GRB + NEO_KHZ800 /**< NeoPixel colour order and signalling speed. */
#define QB_NSECTIONS 6                      /**< Number of theta sections from pole to pole. */
#define QB_NBANDS (QB_NSECTIONS + 1)        /**< Number of horizontal LED bands, including both poles. */
#define QB_NLEGS 12                         /**< Number of meridian legs (phi divisions). */
#define QB_IMU_ADDR 0x6A                    /**< I2C address of the LSM6DS3 IMU. */
#define QB_IX 2                             /**< Chip axis index mapped onto the sphere-frame x axis. */
#define QB_IY 0                             /**< Chip axis index mapped onto the sphere-frame y axis. */
#define QB_IZ 1                             /**< Chip axis index mapped onto the sphere-frame z axis. */
#define QB_SX 0                             /**< If 1, the sphere-frame x axis is sign-flipped. */
#define QB_SY 1                             /**< If 1, the sphere-frame y axis is sign-flipped. */
#define QB_SZ 1                             /**< If 1, the sphere-frame z axis is sign-flipped. */
#define INNER_STATE_COUNT 6                 /**< Maximum number of stored inner Bloch states. */
/** @} */

/** @name LSM6DS3 CTRL8_XL low-pass filter (LPF2) settings
 *  @{ */
#define LSM6DS3_ACC_GYRO_LPF2_XL_EN 0x80             /**< Enable accelerometer LPF2. */
#define LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_50 0x00  /**< LPF2 cut-off at ODR/50. */
#define LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_100 0x20 /**< LPF2 cut-off at ODR/100. */
#define LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_9 0x40   /**< LPF2 cut-off at ODR/9. */
#define LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_400 0x60 /**< LPF2 cut-off at ODR/400. */
                                                     /** @} */

    /** Time each inner state is shown when cycling through states, in ms. */
    static constexpr uint32_t CYCLING_TIME = 600;
    /** Time window in which a local and a remote entanglement request must coincide, in ms. */
    static constexpr uint32_t ENTANGLE_WINDOW_MS = 100;

/** Maximum number of simultaneous BLE peripheral connections. */
#define QB_MAX_PRPH_CONNECTION 2

    /** @name BLE UUIDs (128-bit, little-endian byte order)
     *  @{ */
    /** Qbead primary service UUID. */
    const uint8_t QB_UUID_SERVICE[] =
        {0x45, 0x8d, 0x08, 0xaa, 0xd6, 0x63, 0x44, 0x25, 0xbe, 0x12, 0x9c, 0x35, 0xc6, 0x1f, 0x0c, 0xe3};
    /** Colour characteristic UUID. */
    const uint8_t QB_UUID_COL_CHAR[] =
        {0x45, 0x8d, 0x08, 0xaa, 0xd6, 0x63, 0x44, 0x25, 0xbe, 0x12, 0x9c, 0x35, 0xc6 + 1, 0x1f, 0x0c, 0xe3};
    /** Spherical-coordinates characteristic UUID. */
    const uint8_t QB_UUID_SPH_CHAR[] =
        {0x45, 0x8d, 0x08, 0xaa, 0xd6, 0x63, 0x44, 0x25, 0xbe, 0x12, 0x9c, 0x35, 0xc6 + 2, 0x1f, 0x0c, 0xe3};
    /** Accelerometer characteristic UUID. */
    const uint8_t QB_UUID_ACC_CHAR[] =
        {0x45, 0x8d, 0x08, 0xaa, 0xd6, 0x63, 0x44, 0x25, 0xbe, 0x12, 0x9c, 0x35, 0xc6 + 3, 0x1f, 0x0c, 0xe3};
    /** Tap characteristic UUID. */
    const uint8_t QB_UUID_TAP_CHAR[] =
        {0x45, 0x8d, 0x08, 0xaa, 0xd6, 0x63, 0x44, 0x25, 0xbe, 0x12, 0x9c, 0x35, 0xc6 + 4, 0x1f, 0x0c, 0xe3};
    /** Generic data-packet characteristic UUID. */
    const uint8_t QB_UUID_DATA_CHAR[] =
        {0x45, 0x8d, 0x08, 0xaa, 0xd6, 0x63, 0x44, 0x25, 0xbe, 0x12, 0x9c, 0x35, 0xc6 + 5, 0x1f, 0x0c, 0xe3};
    /** @} */

    /** Twenty zero bytes, e.g. for clearing a BLE characteristic. */
    const uint8_t zerobuffer20[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

    /**
     * @brief State of the shared xorshift32 generator.
     *
     * Defined once in QbeadUtils.cpp so every translation unit shares the same state.
     * Never set it to 0 directly; use seedRNG().
     */
    extern uint32_t stateRNG;

    /** @name Colour helpers (packed 0x00RRGGBB)
     *  @{ */

    /**
     * @brief Packs 8-bit red, green and blue channels into a 24-bit colour.
     * @param r Red channel, 0-255.
     * @param g Green channel, 0-255.
     * @param b Blue channel, 0-255.
     * @return Packed colour 0x00RRGGBB.
     */
    uint32_t color(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Extracts the red channel.
     * @param rgb Packed colour 0x00RRGGBB.
     * @return Red channel, 0-255.
     */
    uint8_t redch(uint32_t rgb);

    /**
     * @brief Extracts the green channel.
     * @param rgb Packed colour 0x00RRGGBB.
     * @return Green channel, 0-255.
     */
    uint8_t greench(uint32_t rgb);

    /**
     * @brief Extracts the blue channel.
     * @param rgb Packed colour 0x00RRGGBB.
     * @return Blue channel, 0-255.
     */
    uint8_t bluech(uint32_t rgb);

    /**
     * @brief Adds two colours channel by channel, saturating each channel at 255.
     * @param c0 First colour.
     * @param c1 Second colour.
     * @return Saturated sum of @p c0 and @p c1.
     */
    uint32_t addColor(uint32_t c0, uint32_t c1);

    /**
     * @brief Scales each channel of a colour linearly, saturating at 255.
     * @param a Scale factor (typically 0-1).
     * @param c Colour to scale.
     * @return Scaled colour.
     */
    uint32_t scaleColor(float a, uint32_t c);

    /**
     * @brief Scales each channel of a colour by the square of a factor, saturating at 255.
     * @param a Scale factor (typically 0-1); the channels are multiplied by a².
     * @param c Colour to scale.
     * @return Scaled colour.
     */
    uint32_t scaleColorQuad(float a, uint32_t c);

    /**
     * @brief Scales each channel of a colour by an 8-bit factor (a/255), saturating at 255.
     * @param a Scale factor, 0-255 (255 = unchanged).
     * @param c Colour to scale.
     * @return Scaled colour.
     */
    uint32_t scaleColor_8bit(uint8_t a, uint32_t c);

    /**
     * @brief Classic red-green-blue colour wheel.
     * @param wheelPos Position on the wheel, 0-255.
     * @return Packed colour at @p wheelPos.
     */
    uint32_t colorWheel(uint8_t wheelPos);

    /**
     * @brief Colour wheel indexed in degrees.
     * @param wheelPos Position on the wheel in degrees, 0-360.
     * @return Packed colour at @p wheelPos.
     */
    uint32_t colorWheel_deg(float wheelPos);
    /** @} */

    /**
     * @brief Parabolic wave for pulsating patterns.
     *
     * Similar to a triangular wave, but the profile resembles a beating heart more
     * closely and can give more pleasing visuals.
     * ![Depiction of the parabolic wave.](./parabola_wave.png)
     *
     * @param x Phase, 0-255 (one full period).
     * @return Wave value, 0-255.
     */
    uint8_t parabolaWave(uint8_t x);

    /**
     * @brief Sign of a value, treating 0 as negative.
     * @param x Input value.
     * @return +1 if @p x > 0, otherwise -1.
     */
    float sign(float x);

    /** @name Spherical coordinates
     *  Convention: z = cos(t), x = cos(p)sin(t), y = sin(p)sin(t).
     *  @{ */

    /**
     * @brief Azimuthal angle of the point (x, y).
     * @param x x coordinate.
     * @param y y coordinate.
     * @return Angle in radians between the x axis and the line to (x, y), in [-pi, pi].
     */
    float phi(float x, float y);

    /**
     * @brief Azimuthal angle of the point (x, y, z); @p z is ignored.
     * @param x x coordinate.
     * @param y y coordinate.
     * @param z z coordinate (unused).
     * @return Angle in radians, in [-pi, pi].
     */
    float phi(float x, float y, float z);

    /**
     * @brief Polar angle of the vector (x, y, z) measured from the +z axis.
     * @param x x coordinate.
     * @param y y coordinate.
     * @param z z coordinate.
     * @return Angle in radians, in [0, pi].
     */
    float theta(float x, float y, float z);
    /** @} */

    /** @name Angle helpers
     *  @{ */

    /**
     * @brief Converts degrees to radians.
     * @param angle Angle in degrees.
     * @return Angle in radians.
     */
    float toRadians(const float angle);

    /**
     * @brief Converts radians to degrees.
     * @param angle Angle in radians.
     * @return Angle in degrees.
     */
    float toDegrees(const float angle);

    /**
     * @brief Sine of an angle given in degrees.
     * @param angle Angle in degrees.
     * @return sin(angle).
     */
    float sin_deg(const float angle);

    /**
     * @brief Cosine of an angle given in degrees.
     * @param angle Angle in degrees.
     * @return cos(angle).
     */
    float cos_deg(const float angle);

    /**
     * @brief Modulo that returns a non-negative result for negative @p a.
     * @param a Dividend.
     * @param b Divisor (expected positive).
     * @return fmod(a, b), shifted by @p b when @p a < 0.
     */
    float modulo(const float a, const float b);

    /**
     * @brief Checks that spherical angles are in range.
     * @param theta Polar angle in degrees; valid range [0, 180].
     * @param phi   Azimuthal angle in degrees; valid range [0, 360].
     * @return True if both angles are in range.
     */
    bool checkThetaAndPhi(float theta, float phi);
    /** @} */

    /**
     * @brief Maps a (leg, pixel) position on the sphere to a linear LED index.
     *
     * The leg direction is inverted because the PCB is laid out as a left-handed
     * coordinate system. Pixel 0 and pixel QB_NSECTIONS are the shared pole pixels.
     *
     * @param leg   Meridian leg index, 0 to QB_NLEGS - 1.
     * @param pixel Pixel along the leg, 0 (north pole) to QB_NSECTIONS (south pole).
     * @return Index into the LED strip.
     */
    int computePixelIndex(int leg, int pixel);

    /**
     * @brief A 3D direction, normalised on construction from components.
     */
    struct Axis
    {
        float x; /**< x component. */
        float y; /**< y component. */
        float z; /**< z component. */

        /** @brief Constructs the +z axis (0, 0, 1). */
        Axis();

        /**
         * @brief Constructs a unit axis from components.
         * @param x x component.
         * @param y y component.
         * @param z z component.
         * @warning A zero vector gives NaN components; no guard is applied.
         */
        Axis(float x, float y, float z);
    };

    /** @name Axis helpers
     *  @{ */

    /**
     * @brief Clamps a value to [-1, 1], e.g. before acos().
     * @param value Input value.
     * @return @p value clamped to [-1, 1].
     */
    float clampUnit(float value);

    /**
     * @brief Normalises an axis in place.
     * @param[in,out] axis Axis to normalise; left untouched on failure.
     * @return False if the norm is not finite or below 1e-6, otherwise true.
     */
    bool normalise(Axis &axis);

    /**
     * @brief Returns the antipodal axis.
     * @param axis Input axis.
     * @return (-x, -y, -z).
     */
    Axis oppositeAxis(const Axis &axis);

    /**
     * @brief Dot product of two axes.
     * @param a First axis.
     * @param b Second axis.
     * @return a · b.
     */
    float dotProduct(const Axis &a, const Axis &b);

    /**
     * @brief Converts an axis to spherical angles in degrees.
     * @param axis Unit axis.
     * @param[out] thetaDeg Polar angle in degrees, [0, 180].
     * @param[out] phiDeg   Azimuthal angle in degrees, [0, 360).
     */
    void axisAngles(const Axis &axis, float &thetaDeg, float &phiDeg);

    /**
     * @brief Builds a unit axis from spherical angles in degrees.
     * @param thetaDeg Polar angle in degrees.
     * @param phiDeg   Azimuthal angle in degrees.
     * @return Unit axis.
     */
    Axis axisFromDegrees(float thetaDeg, float phiDeg);

    /**
     * @brief Returns a unit axis perpendicular to @p axis.
     * @param axis Normalised input axis.
     * @return A normalised axis orthogonal to @p axis.
     * @pre @p axis is normalised.
     */
    Axis perpendicularTo(const Axis &axis);
    /** @} */

    /** @name Random number generation
     *  @{ */

    /**
     * @brief Get a 32-bit seed from the nRF52840 hardware random number generator.
     *
     * When the SoftDevice is running, the bytes come from the SoftDevice's random
     * pool, which is fed by the chip's hardware RNG (a true random source based
     * on thermal noise, not a deterministic algorithm).
     *
     * If the SoftDevice is not running or the pool is empty, falls back to a
     * micros()-based value. That fallback is NOT truly random.
     *
     * Advised to avoid using sd_rand for all your rng calls as it's more error prone
     * and has incosistent call execution times.
     *
     * @return uint32_t Non-zero seed value.
     */
    uint32_t getHardwareSeed();

    /**
     * @brief Deterministically seeds the global RNG state.
     *
     * @param s The seed may not be 0! If a 0 is passed the seed falls back to a constant value.
     */
    void seedRNG(uint32_t s);

    /**
     * @brief Advance the shared xorshift32 generator and return the next value.
     *
     * Marsaglia's xorshift32 (shifts 13, 17, 5). It steps through every non-zero
     * 32-bit value exactly once before repeating, so the period is 2^32 - 1.
     *
     * Fully deterministic when seeded. Seed it by way of seedRNG() (and getHardwareSeed()).
     *
     * @note stateRNG must never be 0. A zero state stays zero forever, so
     *       seedRNG() substitutes a non-zero constant.
     * @note Not cryptographically secure, and the lowest bits are the weakest.
     *       Use the high bits where it is cheap to do so.
     * @note Not safe to call from an interrupt, since it modifies shared state.
     *
     * @return uint32_t The next pseudo-random 32-bit value (never 0).
     */
    uint32_t updateRNG();
    /** @} */
}

#endif // QBEAD_UTILS_H
