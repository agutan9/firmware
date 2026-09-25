#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>
#include <bluefruit.h>
#include <Qbead.h>

#include "color_tools.h"

// #define VERBOSE

#define NORTH_POLE_IDX 0
#define SOUTH_POLE_IDX 6
#define NUM_PIXELS 62

static uint32_t white = Adafruit_NeoPixel::Color(255, 255, 255);

Qbead::Qbead bead;
Qbead::BlochVector local_state;
Qbead::BlochVector shared_state;
Qbead::BlochVector rot_axis;

struct PixelAxis
{
    float x, y, z;
};

// TODO IF ALLOWED TO HARDCODE MAKE static constexpr
PixelAxis pixelLUT[NUM_PIXELS];

// TODO: Remove or rectify
void initPixelLUT(const Qbead::Qbead &bead);
uint32_t mapRedBlackGreenDiscontinuous(float geomInProd);
uint32_t getBaseContourColour(float geomInProd);
void setLocalContourBands(Qbead::Qbead &bead, const Qbead::BlochVector &arbAxis, float damping);
void setEntangledContourBands(Qbead::Qbead &bead, const Qbead::BlochVector &arbAxis, float damping, bool half);

bool gammaToggle = true;
bool entangledToggle = false;
bool fadeToggle = false;
bool fadeFlip = false;
bool rotation = true;
bool half = false;

uint32_t lastFullToggleTime = 0;
uint32_t fadeStartTime = 0;
const uint32_t toggleInterval = 5000;
const uint32_t fadeInterval = 600;
float luxLevel = 1.0f;

void setup()
{
    Serial.begin(9600);
    delay(50);
    // IMU Might not initialize but should fail silently letting us test the LED
    bead.begin();
    bead.pixels.setBrightness(255); // Per documentation this was only inteded for one-time setup usage
    initPixelLUT(bead);
    bead.testPixels();
    delay(50);
    setLocalContourBands(bead, local_state, luxLevel);
    bead.pixels.show();
    delay(100);
    // Use to fake IMU axis and entanglement
    shared_state.setXYZ(1.0f, 0.0f, 0.0f);
    rot_axis.setXYZ(0.0f, 1.0f, 0.0f);
    bead.clear();
}
// TODO: Resolve inverse polarity issue (currently I think because everything is flipped RED maps to -Z not +Z)

// TODO: Maybe add pulsation? The ParabolaWave from Qbead.h is pretty hectic so NeoPixel sine might just be better
// TODO: Fine-tune the Brightness vs gamma (use our own custom gamma table / func for different gammas)

void loop()
{
    delay(100);
    if (rotation)
    {
        local_state.rotateAround(rot_axis, 0.5f);
        shared_state.rotateAround(rot_axis, 0.5f);
    }

    if (!bead.imu.begin())
    {
        Serial.println("IT'S WORKING");
    }
    else {
        Serial.println("IT'S NOT WORKING");
    }

    // Serial.println("Contour axis changed by 3.5f..");
    bead.clear(); // Redundant? as we write to all pixels

    if (half)
        setLocalContourBands(bead, local_state, luxLevel);
    if (entangledToggle)
    {
        setEntangledContourBands(bead, shared_state, luxLevel, half);
    }
    else
    {
        if (!half)
            setLocalContourBands(bead, local_state, luxLevel);
    }
    bead.pixels.show();

    // Pulse in and out

    // TODO: Only one half
    uint32_t now = millis();
    if (fadeToggle)
    {
        uint32_t fadeNow = now - fadeStartTime;
        uint32_t halfInterval = fadeInterval / 2;
        // fadeNow is implicitly % fadeInterval here
        uint8_t phase = (uint8_t)((fadeNow * 255UL) / fadeInterval);
        // Want to start at 1.0 (sin8 starts at 0.5) so need to phase shift to a cosine
        luxLevel = Adafruit_NeoPixel::sine8(phase + 64) / 255.0f;
        if (!fadeFlip && fadeNow >= halfInterval)
        {
            fadeFlip = true; // disable further flips
            entangledToggle = !entangledToggle;
            lastFullToggleTime = now; // Count Interval from fade-in
        }
        if (fadeNow >= fadeInterval)
        { // Reset and exit fade sub-loop
            fadeFlip = false;
            fadeToggle = false;
            luxLevel = 1.0f;
        }
    }
    else
    { // toggleInterval counts from the moment of fading in
        if (now - lastFullToggleTime >= toggleInterval)
        {
            fadeToggle = true;
            fadeStartTime = now;
        }
    }
}

// TODO: Could use a hardcoded LU. Perhaps with a check if the assumed tot# of pixels is still the same(?)
// Codestyle question
void initPixelLUT(const Qbead::Qbead &bead)
{
    // Trivial: Do the poles seperately
    // DUE TO BUG OF REVERSED FIRST ORDER I HAVE EVERYTHING REVERSED NOW
    // TODO: SET RIGHT

    //// Northpole aligned with +Z unit vector
    // pixelLUT[0] = {0.0, 0.0,  1.0};
    //// Southpole aligned with -Z unit vector
    // pixelLUT[6] = {0.0, 0.0, -1.0};
    //// Non-trivial: All the unique-per-leg pixels

    // Northpole aligned with -Z unit vector
    pixelLUT[0] = {0.0, 0.0, -1.0};
    // Southpole aligned with +Z unit vector
    pixelLUT[6] = {0.0, 0.0, +1.0};
    // Non-trivial: All the unique-per-leg pixels

    // float band_theta = bead.theta_quant;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // TODO: BUG The first leg has reverse indexation order as all others!!!
    // Applying a reverse order hack here for now, but ideally this gets resolved
    // float cum_theta = 0.0f;
    float cum_theta = bead.nsections * bead.theta_quant;
    // Do the first leg seperately because it has the 6 pole issue
    for (int fleg_th = 1; fleg_th < bead.nsections; ++fleg_th)
    {
        //
        // cum_theta += bead.theta_quant;
        cum_theta -= bead.theta_quant;
#ifdef VERBOSE
        x = Qbead::sin_deg(cum_theta);
        y = 0.0f;
        z = Qbead::cos_deg(cum_theta);
        Serial.printf("Pixel ID[%d] Coords: {X:%.2f, Y:%.2f, Z:%.2f}\n", fleg_th, x, y, z);
#endif
        //
        pixelLUT[fleg_th] = {
            Qbead::sin_deg(cum_theta), // X = cos(p)sin(t)
            0.0f,                      // Y = sin(p)sin(t)
            Qbead::cos_deg(cum_theta)  // Z = cos(t)
        };
    }
    // Now we can shift the other legs correctly
    float band_sin_theta = 0.0f;
    float leg_cos_phi = 0.0f;
    float leg_sin_phi = 0.0f;
    int shift = bead.nsections - 1;
    int p_i = 1;

    for (int ph_j = 1; ph_j < bead.nlegs; ++ph_j)
    {
        leg_cos_phi = Qbead::cos_deg(ph_j * bead.phi_quant);
        leg_sin_phi = Qbead::sin_deg(ph_j * bead.phi_quant);
        cum_theta = 0.0f;                                  // desynced from th_i count; use cum. counter
        for (int th_i = 2; th_i <= bead.nsections; ++th_i) // +1 offset from 0 also
        {
            p_i = th_i + ph_j * shift;
            //
            cum_theta += bead.theta_quant;
            band_sin_theta = Qbead::sin_deg(cum_theta);
#ifdef VERBOSE
            x = leg_cos_phi * band_sin_theta;
            y = leg_sin_phi * band_sin_theta;
            z = Qbead::cos_deg(cum_theta);
            Serial.printf("Pixel ID[%d] Coords: {X:%.2f, Y:%.2f, Z:%.2f}\n", p_i, x, y, z);
#endif
            pixelLUT[p_i] = {
                leg_cos_phi * band_sin_theta, // X = cos(p)sin(t)
                leg_sin_phi * band_sin_theta, // Y = sin(p)sin(t)
                Qbead::cos_deg(cum_theta)     // Z = cos(t)
            };
        }
    }
}

uint32_t mapCorrelationDiscontinuous(float geomInProd, float luxScale)
{
    float mag = geomInProd;
    if (geomInProd <= 0)
        mag *= -1;
    int idx = 0.0f;
    if (mag < 0.05f)
        idx = 4; // ~0.0 (orthogonal to axis)
    else if (mag < 0.25f)
        idx = 3; // ~0.13 (small angle off orthognal)
    else if (mag < 0.7f)
        idx = 2; // ~0.50 (45deg)
    else if (mag < 0.95f)
        idx = 1; // ~0.87 (smal angle off parallel)
    else
        idx = 0;
    //
    HSVBand band_HSV = correlationBlueYellowBands[idx];
    // HSVBand band_HSV = (geomInProd >= 0.0f) ? correlationBlueVioletBands[rev_idx] : correlationVioletMagentaBands[rev_idx];
    return Adafruit_NeoPixel::ColorHSV(band_HSV.hue, band_HSV.sat, band_HSV.val * luxScale);
}

// Discontinuous red-green-black map, e.g. 5 bands each side (tune counts/colors to taste)
uint32_t mapRedBlackGreenDiscontinuous(float geomInProd, float luxScale)
{
    // Contrast steps: 0, 20, 80, 160, 255
    // Pure Single Channel w/ manual Gamma correct
    // static constexpr uint32_t redBands[5]   = { 0x00000, 0x140000, 0x500000, 0xA00000, 0xFF0000 };
    // static constexpr uint32_t greenBands[5] = { 0x00000, 0x001400, 0x005000, 0x00A000, 0x00FF00 };
    // HSL -> Blue shift
    // static constexpr uint32_t redBands[5]   = { 0x00001F, 0x3D0052, 0x8F006B, 0xCC0033, 0xFF0000 }; // idx0: near-black blue -> idx4: pure red
    // static constexpr uint32_t greenBands[5] = { 0x00001F, 0x002952, 0x008F8F, 0x00CC66, 0x00FF00 }; // idx0: near-black blue (same as red's) -> idx4: pure green

    float mag = geomInProd;
    if (geomInProd <= 0)
        mag *= -1;
    int idx = 0.0f;
    if (mag < 0.05f)
        idx = 0; // ~0.0 (orthogonal to axis)
    else if (mag < 0.25f)
        idx = 1; // ~0.13 (small angle off orthognal)
    else if (mag < 0.7f)
        idx = 2; // ~0.50 (45deg)
    else if (mag < 0.95f)
        idx = 3; // ~0.87 (smal angle off parallel)
    else
        idx = 4; // ~1.00 (parallel to axis)
#ifdef VERBOSE
    Serial.printf("In Prod[%.2f] gives mag: %.2f and idx: %d\n", geomInProd, mag, idx);
#endif
    // return (geomInProd >= 0.0f) ? redBands[idx] : greenBands[idx];
    //  TODO: reverse order
    uint8_t rev_idx = 4 - idx;
    HSVBand band_HSV = (geomInProd >= 0.0f) ? redYellowBands[rev_idx] : greenYellowBands[rev_idx];
    return Adafruit_NeoPixel::ColorHSV(band_HSV.hue, band_HSV.sat, band_HSV.val * luxScale);
}

// TODO: Currently archaic and continuous
uint32_t mapRedBlackGreenContinuous(uint8_t colorVal)
{
    // 0 = full green, 128 = black, 255 = full red
    if (colorVal < 128)
    {
        uint8_t g = (colorVal - 128) * 2;         // 0 -> 255 (clamps at 254)
        return Adafruit_NeoPixel::Color(0, g, 0); // TODO: Could add +1 but then never 0,0,0
    }
    else
    {
        uint8_t r = 255 - colorVal * 2; // 255 -> 0
        return Adafruit_NeoPixel::Color(r, 0, 0);
    }
}

// TODO: If decide the colour maps should take input 0..255
// refactor them for that and you will need this to convert your InProds
float inProdToColorPos(float geomInProd)
{
    // Contour distance (inner prod) in [-1..+1]. Need to map to [0-255]
    // Avoid 'round' for performance reasons -> add 0.5f
    // Note! For the Northpole LED we get 255.5 which should truncate to 255 but
    // risks a float-rounding overflow to 256->0 getting the anti-polar color assigned
    // TODO: Check if we need catch for North Polar Led due to inverted colour
    uint8_t colorPos = (uint8_t)(geomInProd * 127.5f + 128.0f);
    return colorPos;
}

// TODO: Add customizability of contour map selection (ENUM probably?)
uint32_t getBaseContourColour(float geomInProd, float luxScale, bool gammaCorrect = true)
{
    //
    uint32_t colorMapped = mapRedBlackGreenDiscontinuous(geomInProd, luxScale);
    return gammaCorrect ? Adafruit_NeoPixel::gamma32(colorMapped) : colorMapped;
}

uint32_t getEntangledContourColour(float geomInProd, float luxScale, bool gammaCorrect = true)
{
    // TODO: JUST SHOWCASE. Instead of plane geomInProd would need some kind of correlation calculation
    uint32_t colorMapped = mapCorrelationDiscontinuous(geomInProd, luxScale);
    return gammaCorrect ? Adafruit_NeoPixel::gamma32(colorMapped) : colorMapped;
}

void setLocalContourBands(Qbead::Qbead &bead, const Qbead::BlochVector &arbAxis, float luxScale)
{
    //
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float sum = 0.0f;

    for (int p_i = 0; p_i < NUM_PIXELS; p_i++)
    {
        x = pixelLUT[p_i].x * arbAxis.x;
        y = pixelLUT[p_i].y * arbAxis.y;
        z = pixelLUT[p_i].z * arbAxis.z;
        sum = x + y + z;
#ifdef VERBOSE
        Serial.printf("Pixel ID[%d] InProd: {X:%.2f, Y:%.2f, Z:%.2f} Sum: %.2f\n", p_i, x, y, z, sum);
#endif

        bead.pixels.setPixelColor(p_i, getBaseContourColour(
                                           pixelLUT[p_i].x * arbAxis.x +
                                               pixelLUT[p_i].y * arbAxis.y +
                                               pixelLUT[p_i].z * arbAxis.z
                                           // in-product with pixel's basis unit vecs
                                           ,
                                           luxScale, gammaToggle));
    }
}

void setEntangledContourBands(Qbead::Qbead &bead, const Qbead::BlochVector &arbAxis, float luxScale, bool half = false)
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float sum = 0.0f;

    // TODO :Probbaly remove as it doesnt seem like it could work with this few pixels
    const uint8_t pixelCount = half ? (((NUM_PIXELS - 2) / 2) + 2) : NUM_PIXELS;

    for (int p_i = 0; p_i < pixelCount; p_i++)
    {
        if (half)
        {
            if (p_i == 0 || p_i == 6)
                p_i++;
        }

        x = pixelLUT[p_i].x * arbAxis.x;
        y = pixelLUT[p_i].y * arbAxis.y;
        z = pixelLUT[p_i].z * arbAxis.z;
        sum = x + y + z;
#ifdef VERBOSE
        Serial.printf("Pixel ID[%d] InProd: {X:%.2f, Y:%.2f, Z:%.2f} Sum: %.2f\n", p_i, x, y, z, sum);
#endif

        bead.pixels.setPixelColor(p_i, getEntangledContourColour(
                                           pixelLUT[p_i].x * arbAxis.x +
                                               pixelLUT[p_i].y * arbAxis.y +
                                               pixelLUT[p_i].z * arbAxis.z
                                           // in-product with pixel's basis unit vecs
                                           ,
                                           luxScale, gammaToggle));
    }
}

// SECTION Testing and Utility
//  TODO: Remove and maybe add to Utils?
void showPixelIdsOneByOne(Qbead::Qbead &bead, uint8_t waitTime, uint8_t lastID)
{
    // Showcases the ordering of the pixel ID's
    int counter = 0;
    for (int t = 0; t < lastID; ++t)
    {
        delay(waitTime);
        bead.clear();
        bead.pixels.setPixelColor(t, white);
        bead.show();
        Serial.print("Showing only Pixel ID: ");
        Serial.println(counter);
        counter++;
    }
}
//! SECTION

// QBEADS is visualised in discretized contour bands around the BlochV axis
// If axis = z then it's 7 bands (inc. 2 singular pole bands)
// #pixels: nlegs * (nsections - 1) + 2
// +2 is the poles
// sections is bands in between pixels
// 5 pixels per leg (excl. poles)
// except for singularityies
// -> every contour will be 12 pixels if Z-aligned
// -> Contours not so aligned
// Since its inherently discretized I could also
// calculate the color by some formula of the angle
// Theta is half-sphere angle
// Phi is full-sphere angle
// Need to turn any pixel into a BlochVector / coordinate
// acos(InnerProductGeom) is faster version of the intuitively more correct centralAngle
// nvm that, using [-1..1] is totally fine
