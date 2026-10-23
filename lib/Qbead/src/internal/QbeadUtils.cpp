/**
 * @file QbeadUtils.cpp
 * @brief Implementation of the shared Qbead helpers declared in QbeadUtils.h.
 */
#include "QbeadUtils.h"

namespace Qbead
{
    uint32_t stateRNG = 0x9E3779B9u; // global to ensure rng consistancy across libs

    // TODO manage namespaces better
    uint32_t color(uint8_t r, uint8_t g, uint8_t b)
    {
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }

    uint8_t redch(uint32_t rgb)
    {
        return rgb >> 16;
    }

    uint8_t greench(uint32_t rgb)
    {
        return (0x00ff00 & rgb) >> 8;
    }

    uint8_t bluech(uint32_t rgb)
    {
        return 0x0000ff & rgb;
    }

    uint32_t addColor(uint32_t c0, uint32_t c1)
    {
        uint8_t r = min(0xff, (int)redch(c0) + redch(c1));
        uint8_t g = min(0xff, (int)greench(c0) + greench(c1));
        uint8_t b = min(0xff, (int)bluech(c0) + bluech(c1));
        return color(r, g, b);
    }

    uint32_t scaleColor(float a, uint32_t c)
    {
        uint8_t r = min(0xff, a * redch(c));
        uint8_t g = min(0xff, a * greench(c));
        uint8_t b = min(0xff, a * bluech(c));
        return color(r, g, b);
    }

    uint32_t scaleColorQuad(float a, uint32_t c)
    {
        float a2 = a * a;
        uint8_t r = min(0xff, a2 * redch(c));
        uint8_t g = min(0xff, a2 * greench(c));
        uint8_t b = min(0xff, a2 * bluech(c));
        return color(r, g, b);
    }

    uint32_t scaleColor_8bit(uint8_t a, uint32_t c)
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

    float toRadians(const float angle)
    {
        return angle / 180 * M_PI;
    }

    float toDegrees(const float angle)
    {
        return angle * 180 / M_PI;
    }

    float sin_deg(const float angle)
    {
        return sin(toRadians(angle));
    }

    float cos_deg(const float angle)
    {
        return cos(toRadians(angle));
    }

    // Mod function that only returns positive numbers
    float modulo(const float a, const float b)
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

    Axis::Axis()
    {
        x = 0;
        y = 0;
        z = 1;
    }

    Axis::Axis(float x, float y, float z)
    {
        float lenght = sqrt(x * x + y * y + z * z);
        this->x = x / lenght;
        this->y = y / lenght;
        this->z = z / lenght;
    }

    float clampUnit(float value)
    {
        if (value < -1.f)
            return -1.f;

        if (value > 1.f)
            return 1.f;

        return value;
    }

    bool normalise(Axis &axis)
    {
        const float norm = sqrtf(
            axis.x * axis.x +
            axis.y * axis.y +
            axis.z * axis.z);

        if (!isfinite(norm) || norm < 1e-6f)
            return false;

        axis.x /= norm;
        axis.y /= norm;
        axis.z /= norm;

        return true;
    }

    Axis oppositeAxis(const Axis &axis)
    {
        return {-axis.x, -axis.y, -axis.z};
    }

    float dotProduct(const Axis &a, const Axis &b)
    {
        return a.x * b.x +
               a.y * b.y +
               a.z * b.z;
    }

    void axisAngles(
        const Axis &axis,
        float &thetaDeg,
        float &phiDeg)
    {
        thetaDeg = acosf(clampUnit(axis.z)) * RAD_TO_DEG;
        phiDeg = atan2f(axis.y, axis.x) * RAD_TO_DEG;

        if (phiDeg < 0.f)
            phiDeg += 360.f;
    }

    Axis axisFromDegrees(float thetaDeg, float phiDeg)
    {
        const float t = thetaDeg * DEG_TO_RAD;
        const float p = phiDeg * DEG_TO_RAD;
        const float s = sinf(t);

        return {
            s * cosf(p),
            s * sinf(p),
            cosf(t)};
    }

    Axis perpendicularTo(const Axis &axis)
    {
        // Expects a normalized axis.
        Axis reference = {1.f, 0.f, 0.f};

        if (fabsf(axis.x) > 0.9f)
            reference = {0.f, 0.f, 1.f};

        const float projection = dotProduct(reference, axis);

        Axis perpendicular = {
            reference.x - projection * axis.x,
            reference.y - projection * axis.y,
            reference.z - projection * axis.z};

        normalise(perpendicular);
        return perpendicular;
    }

    uint32_t getHardwareSeed()
    {
        uint8_t softDeviceOn = 0;
        uint32_t seed = 0;
        if (
            sd_softdevice_is_enabled(&softDeviceOn) == NRF_SUCCESS &&
            softDeviceOn &&
            sd_rand_application_vector_get((uint8_t *)&seed, sizeof(seed)) == NRF_SUCCESS)
        {
            // Fall back to constant seed in the rare case we pull 0
            seed = seed ? seed : 0x9E3779B9u;
            Serial.println("[INFO]{RNG} BLE-Stack active and running. Retrieved true random seed!");
            Serial.printf("[INFO]{RNG} Seed: %u\n", seed);
        }
        else
        {
            // In principle one can access the RNG register directly when SoftDevice is off
            // When it is on, this becomes protected memory. However, because the Qbead should
            // always have the BLE stack on we opt for a simple pseudo-random fallback here.
            seed = (micros() ^ 0xA5A5A5A5u) | 1u; // fallback, never 0
            Serial.println("[INFO]{RNG} Error in accessing RNG per BLE-stack. Falling back to pseudo-random seed!");
            Serial.printf("[INFO]{RNG} Seed: %u\n", seed);
            // micros near boot will be very similar each time! beware
        }
        return seed;
    }

    void seedRNG(uint32_t s) { stateRNG = s ? s : 0x9E3779B9u; }

    uint32_t updateRNG()
    {
        stateRNG ^= stateRNG << 13;
        stateRNG ^= stateRNG >> 17;
        stateRNG ^= stateRNG << 5;
        return stateRNG;
    }
}
