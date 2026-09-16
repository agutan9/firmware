#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>
#include <bluefruit.h>
#include <Qbead.h>

#include "color_tools.h"

//#define VERBOSE

#define NORTH_POLE_IDX 0
#define SOUTH_POLE_IDX 6
#define NUM_PIXELS 62

static uint32_t white = Adafruit_NeoPixel::Color(255, 255, 255);

Qbead::Qbead bead;
Qbead::BlochVector state;
Qbead::BlochVector test_rot_axis;

struct PixelAxis {
    float x, y, z;
};

// TODO IF ALLOWED TO HARDCODE MAKE static constexpr
PixelAxis pixelLUT[NUM_PIXELS];

// TODO: Remove or rectify
void initPixelLUT(const Qbead::Qbead &bead);
uint32_t mapRedBlackGreenDiscontinuous(float geomInProd);
uint32_t getContourColour(float geomInProd);
void setContourBands(Qbead::Qbead &bead, const Qbead::BlochVector &arbAxis);

// TODO: Resolve inverse polarity issue (currently I think because everything is flipped RED maps to -Z not +Z)
// --- Sweep candidate lists -------------------------------------------
// idx0-idx2 "val" candidates (idx3/idx4 stay at their original table value
// as fixed anchors so you always have a stable bright reference on-sphere).
static const uint8_t valCandidates[] = { 15, 20, 25, 31, 45, 60, 90, 120 };
static const uint8_t NUM_VAL = sizeof(valCandidates) / sizeof(valCandidates[0]);

// Global saturation candidates (applied to all 5 bands equally this round;
// split into per-band arrays later if you want independent control).
static const uint8_t satCandidates[] = { 255, 180, 90, 60 };
static const uint8_t NUM_SAT = sizeof(satCandidates) / sizeof(satCandidates[0]);

static uint8_t valStep = 0;
static uint8_t satStep = 0;

uint8_t gammaLUT[256];
bool gammaToggle = false;
// --- Sweep

void setup() {
    Serial.begin(9600);
    delay(50);
    // IMU Might not initialize but should fail silently letting us test the LED
    bead.begin();
    bead.pixels.setBrightness(255); // Per documentation this was only inteded for one-time setup usage
    initPixelLUT(bead);
    bead.testPixels();
    delay(50);
    setContourBands(bead, state);
    bead.pixels.show();
    delay(100);
    // Use to fake IMU axis
    test_rot_axis.setXYZ(0.0f, 1.0f, 0.0f);
    bead.clear();

    // Color Fine tuning
    buildGammaTable(gammaLUT, 2.8f);   // start steeper than default 2.8
}

void loop() {  
    delay(100);
    //state.rotateAround(test_rot_axis, 1.5f);
    bead.clear(); // Redundant? as we write to all pixels
// Pulsate
    // TODO: WIP -> Maybe just use the innante NeoPixel sine func?
    //const uint8_t period = 5000;
    //uint32_t t = millis() % period;
    //uint8_t x = (uint8_t)((t * 255UL) / period);
    //uint8_t wave = Qbead::parabolaWave(x);
    //uint8_t brightness = (uint8_t)((wave * 20UL) / 252);
    //brightness = min((uint8_t)5, brightness); // never fully off, adjust min as desired
    //bead.pixels.setBrightness(brightness);
  
    setContourBands(bead, state);
    bead.pixels.show();
    //Serial.println("Contour axis changed by 3.5f.."); // TODO: REMOVE

    delay(3000);
    gammaToggle = gammaToggle ? false : true;

    //// Color finetuning
    //uint8_t testVal = valCandidates[valStep];
    //uint8_t testSat = satCandidates[satStep];
    ////bead.clear();
    //for (int i = 4; i >= 0; i--) {
    //    HSVBand b = redBandsHSV[i];   // copy so the source table stays untouched
//
    //    // Only override val on the low, hard-to-see bands (idx0-idx2).
    //    // idx3/idx4 keep their original val as a fixed bright anchor.
    //    if (i <= 2) {
    //        b.val = testVal;
    //    }
    //    b.sat = testSat;
//
    //    uint32_t raw = Adafruit_NeoPixel::ColorHSV(b.hue, b.sat, b.val);
    //    uint32_t corrected = 0;
    //    if (gammaToggle)
    //    {
    //        uint8_t r = gammaLUT[(raw >> 16) & 0xFF];
    //        uint8_t g = gammaLUT[(raw >> 8) & 0xFF];
    //        uint8_t bch = gammaLUT[raw & 0xFF];
    //        corrected = ((uint32_t)r << 16) | ((uint32_t)g << 8) | bch;
    //    }
    //    else 
    //    {
    //        corrected = raw;
    //    }
//
//
    //    bead.pixels.setPixelColor(i, corrected);
    //}
    //bead.pixels.show();
//
    //// Print what's currently on the sphere so you can log the combo that
    //// looked best without having to guess from memory afterwards.
    //Serial.print("valStep=");   Serial.print(valStep);
    //Serial.print(" testVal=");  Serial.print(testVal);
    //Serial.print("  satStep="); Serial.print(satStep);
    //Serial.print(" testSat=");  Serial.println(testSat);
//
    //delay(3500);
    //// --- Advance counters: sat cycles fully before val advances ---
    //satStep++;
    //if (satStep >= NUM_SAT) {
    //    satStep = 0;
    //    valStep++;
    //    if (valStep >= NUM_VAL) {
    //        valStep = 0;   // wrap around and repeat the whole grid
    //    }
    //}
}

// TODO: Could use a hardcoded LU. Perhaps with a check if the assumed tot# of pixels is still the same(?)
// Codestyle question 
void initPixelLUT(const Qbead::Qbead &bead)
{
    // Trivial: Do the poles seperately
    // DUE TO BUG OF REVERSED FIRST ORDER I HAVE EVERYTHING REVERSED NOW 
    // TODO: SET RIGHT

    //// Northpole aligned with +Z unit vector
    //pixelLUT[0] = {0.0, 0.0,  1.0};
    //// Southpole aligned with -Z unit vector
    //pixelLUT[6] = {0.0, 0.0, -1.0};
    //// Non-trivial: All the unique-per-leg pixels

    // Northpole aligned with -Z unit vector
    pixelLUT[0] = {0.0, 0.0,  -1.0};
    // Southpole aligned with +Z unit vector
    pixelLUT[6] = {0.0, 0.0, +1.0};
    // Non-trivial: All the unique-per-leg pixels

    //float band_theta = bead.theta_quant;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // TODO: BUG The first leg has reverse indexation order as all others!!!
    // Applying a reverse order hack here for now, but ideally this gets resolved
    // float cum_theta = 0.0f;
    float cum_theta = bead.nsections * bead.theta_quant;
    // Do the first leg seperately because it has the 6 pole issue
    for (int fleg_th = 1; fleg_th < bead.nsections; ++fleg_th){
        //
        //cum_theta += bead.theta_quant;
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
        cum_theta = 0.0f; // desynced from th_i count; use cum. counter
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

// Discontinuous red-green-black map, e.g. 5 bands each side (tune counts/colors to taste)
uint32_t mapRedBlackGreenDiscontinuous(float geomInProd)
{
    // Contrast steps: 0, 20, 80, 160, 255
    // Pure Single Channel w/ manual Gamma correct
    //static constexpr uint32_t redBands[5]   = { 0x00000, 0x140000, 0x500000, 0xA00000, 0xFF0000 };
    //static constexpr uint32_t greenBands[5] = { 0x00000, 0x001400, 0x005000, 0x00A000, 0x00FF00 };
    // Manual Gamma correct + Muddle attempt
    //static constexpr uint32_t redBands[5]   = { 0x00000, 0x140502, 0x501004, 0xA20008, 0xFF0000 };
    //static constexpr uint32_t greenBands[5] = { 0x00000, 0x051402, 0x105004, 0x20A008, 0x00FF00 };
    // HSL -> Blue shift
    //static constexpr uint32_t redBands[5]   = { 0x00001F, 0x3D0052, 0x8F006B, 0xCC0033, 0xFF0000 }; // idx0: near-black blue -> idx4: pure red
    //static constexpr uint32_t greenBands[5] = { 0x00001F, 0x002952, 0x008F8F, 0x00CC66, 0x00FF00 }; // idx0: near-black blue (same as red's) -> idx4: pure green
    
    // Using color tools for tuning



    float mag = geomInProd;
    if (geomInProd <= 0) mag *= -1;
    int idx = 0.0f;
    if (mag < 0.05f) idx = 0;        // ~0.0 (orthogonal to axis)
    else if (mag < 0.25f) idx = 1;   // ~0.13 (small angle off orthognal)
    else if (mag < 0.7f) idx = 2;    // ~0.50 (45deg)
    else if (mag < 0.95f) idx = 3;   // ~0.87 (smal angle off parallel)
    else idx = 4;                    // ~1.00 (parallel to axis)
#ifdef VERBOSE
    Serial.printf("In Prod[%.2f] gives mag: %.2f and idx: %d\n", geomInProd, mag, idx);
#endif
    //return (geomInProd >= 0.0f) ? redBands[idx] : greenBands[idx];
    // TODO: reverse order
    uint8_t rev_idx = 4 - idx;
    HSVBand band_HSV = (geomInProd >= 0.0f) ? redYellowBands[rev_idx] : greenYellowBands[rev_idx];
    return Adafruit_NeoPixel::ColorHSV(band_HSV.hue, band_HSV.sat, band_HSV.val);
}

// TODO: Currently archaic and continuous
uint32_t mapRedBlackGreenContinuous(uint8_t colorVal)
{ 
    // 0 = full green, 128 = black, 255 = full red
    if (colorVal < 128)
    {
        uint8_t g = (colorVal - 128) * 2; // 0 -> 255 (clamps at 254)
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
}

// TODO: Add customizability of contour map selection (ENUM probably?)
uint32_t getContourColour(float geomInProd, bool gammaCorrect = true) 
{
    //
    uint32_t colorMapped =  mapRedBlackGreenDiscontinuous(geomInProd);
    return gammaCorrect ? Adafruit_NeoPixel::gamma32(colorMapped) : colorMapped;
}

void setContourBands(Qbead::Qbead &bead, const Qbead::BlochVector &arbAxis)
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

        bead.pixels.setPixelColor(p_i, getContourColour(
            pixelLUT[p_i].x * arbAxis.x +
            pixelLUT[p_i].y * arbAxis.y +
            pixelLUT[p_i].z * arbAxis.z
            // in-product with pixel's basis unit vecs
            , false
        ));
    }
}


//SECTION - 




//SECTION Testing and Utility
// TODO: Remove and maybe add to Utils?
void showPixelIdsOneByOne(Qbead::Qbead &bead, uint8_t waitTime, uint8_t lastID)
{
  //Showcases the ordering of the pixel ID's
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
//!SECTION

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
