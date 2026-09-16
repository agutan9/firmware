// qbead_saturation_sweep.ino
// -----------------------------------------------------------------------
// Isolated saturation-only sweep, single fixed hue (pure red), fixed val.
// No gamma correction, brightness fixed at 255 to match your current
// test conditions. Only one counter this time since hue and val are held
// constant, so you can attribute any "white-out" purely to saturation.
//
// Only pixel index 0 is driven so you can also rule out diffuser
// cross-talk between adjacent pixels (see comment at bottom for the
// single-vs-all-pixel comparison test).
// -----------------------------------------------------------------------
#include <Arduino.h>
#include <LSM6DS3.h>
#include <math.h>
#include <bluefruit.h>
#include <Qbead.h>
#include <Adafruit_NeoPixel.h>

#include "color_tools.h"

Qbead::Qbead bead;


// ===========================================================================
// RED -> YELLOW -> BLACK
// One bipolar-hemisphere pole. idx0 = dark red anchor, idx4 = bright
// yellow-orange anchor. hue16=6554 (36 deg) at idx3 matches your liked 6553.
// ===========================================================================
static constexpr HSVBand redYellowBands[5] = {
    {0,     255, 255},  // idx0: near-black red
    {1820,  255, 210},  // idx1: 10 deg
    {4005,  255, 150},  // idx2: 22 deg
    {6554,  255, 90},   // idx3: 36 deg  (your liked hue)
    {9102,  255, 20},   // idx4: 50 deg, bright yellow-orange
};

// ===========================================================================
// GREEN -> YELLOW -> BLACK
// Other bipolar-hemisphere pole. idx0 = dark green anchor, idx4 shares the
// same yellow endpoint as redYellowBands (intentional - see note below).
// hue16=18204 (100 deg) at idx1 is close to your liked 19960 (109.6 deg).
// ===========================================================================
static constexpr HSVBand greenYellowBands[5] = {
    {21845, 255, 255},   // idx0: near-black green, 120 deg
    {18204, 255, 210},   // idx1: 100 deg (near your liked 19960)
    {15474, 255, 150},  // idx2: 85 deg
    {11833, 255, 90},  // idx3: 65 deg
    {9102,  255, 20},  // idx4: 50 deg, bright yellow (shared anchor w/ red map)
};
// NOTE: idx4 hue is intentionally identical between redYellowBands and
// greenYellowBands, so "bright/high-magnitude" reads as a related color
// language on both poles. If you want the two maps fully disjoint instead,
// change greenYellowBands[4].hue to something like hue16(75) (~75 deg,
// yellow-green) so it never touches the red map's yellow.

// ===========================================================================
// CORRELATION BEADS - OPTION A: BLUE -> BLUE-VIOLET
// hue16=39321 (216 deg) at idx0 and 45874 (252 deg) at idx3 are both from
// your liked list.
// ===========================================================================
static constexpr HSVBand correlationBlueVioletBands[5] = {
    {39321, 255, 60},   // idx0: 216 deg  (your liked hue)
    {41506, 255, 120},  // idx1: 228 deg
    {43690, 255, 180},  // idx2: 240 deg, pure blue
    {45874, 255, 230},  // idx3: 252 deg  (your liked hue)
    {49151, 255, 255},  // idx4: 270 deg, blue-violet bright anchor
};

// ===========================================================================
// CORRELATION BEADS - OPTION B: VIOLET -> MAGENTA
// hue16=58981 (324 deg) at idx4 is from your liked list. Picks up where
// Option A leaves off (270 deg) so the two options can be used together
// as one continuous 10-step ramp (216 -> 324 deg) if you need to
// distinguish two correlation-bead types on the same sphere.
// ===========================================================================
static constexpr HSVBand correlationVioletMagentaBands[5] = {
    {49151, 255, 60},   // idx0: 270 deg
    {52428, 255, 120},  // idx1: 288 deg
    {54612, 255, 180},  // idx2: 300 deg
    {56797, 255, 230},  // idx3: 312 deg
    {58982, 255, 255},  // idx4: 324 deg  (your liked hue)
};

// ===========================================================================
// Usage: identical pattern to redBandsHSV from qbead_color_bands.h
// ===========================================================================
//   for (int i = 0; i < 5; i++) {
//       const HSVBand &b = redYellowBands[i];   // or any of the 4 tables
//       uint32_t raw = Adafruit_NeoPixel::ColorHSV(b.hue, b.sat, b.val);
//       bead.pixels.setPixelColor(i, raw);
//   }
//
// Suggested next test: run redYellowBands and correlationBlueVioletBands
// simultaneously on adjacent bead groups (not just isolated single-bead
// sweeps) to confirm hue separation holds up under real multi-bead
// shell-mixing conditions, since that's the actual operating scenario.

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
bool gammaToggle = true;
// --- Sweep



// Fixed hue = pure red (0 deg -> hue16 = 0), fixed val = full (255).
static const uint8_t FIXED_VAL = 255;
static const uint8_t FIXED_SAT = 255;

// Saturation candidates, high to low, so you watch it desaturate step by
// step rather than jumping around. Edit freely / add finer steps once you
// find the region where "white" first becomes "tinted."
static const uint16_t hueCandidates[] = { 65535, 58981, 52428, 45874, 39321, 32767, 26214, 19660, 13107, 6553, 0 };
static const uint8_t NUM_HUE = sizeof(hueCandidates) / sizeof(hueCandidates[0]);

static uint8_t hueStep = 0;

void setup() {
    Serial.begin(9600);
    bead.pixels.begin();
    bead.pixels.setBrightness(255);   // matches your current test condition

    // Color Fine tuning
    buildGammaTable(gammaLUT, 2.8f);   // start steeper than default 2.8
}
// USE THIS FOR INSPECTING NEW COLOR MAPS
void loop(){
    for (int i = 0; i < 5; i++) {
        const HSVBand &band1 = redYellowBands[i];   // or any of the 4 tables
        uint32_t raw = Adafruit_NeoPixel::ColorHSV(band1.hue, band1.sat, band1.val);
        raw = Adafruit_NeoPixel::gamma32(raw);
        bead.pixels.setPixelColor(i, raw);
        uint8_t r = (raw >> 16) & 0xFF;
        uint8_t g = (raw >> 8) & 0xFF;
        uint8_t b = raw & 0xFF;
        Serial.print("pixID="); Serial.print(i);
        Serial.print(" hue=");    Serial.print(band1.hue);
        Serial.print("  R=");     Serial.print(r);
        Serial.print(" G=");      Serial.print(g);
        Serial.print(" B=");      Serial.println(b);
    }

    bead.pixels.show();
    delay(3000);
    bead.clear();
    bead.pixels.show();
    delay(1000);

    for (int i = 0; i < 5; i++) {
        const HSVBand &band2 = greenYellowBands[i];   // or any of the 4 tables
        uint32_t raw = Adafruit_NeoPixel::ColorHSV(band2.hue, band2.sat, band2.val);
        raw = Adafruit_NeoPixel::gamma32(raw);
        bead.pixels.setPixelColor(i, raw);
        uint8_t r = (raw >> 16) & 0xFF;
        uint8_t g = (raw >> 8) & 0xFF;
        uint8_t b = raw & 0xFF;
        Serial.print("pixID="); Serial.print(i);
        Serial.print(" hue=");    Serial.print(band2.hue);
        Serial.print("  R=");     Serial.print(r);
        Serial.print(" G=");      Serial.print(g);
        Serial.print(" B=");      Serial.println(b);
    }
    bead.pixels.show();
    delay(3000);
    bead.clear();
    bead.pixels.show();
    delay(1000);

    for (int i = 0; i < 3; i++) {
        const HSVBand &band3 = redYellowBands[i];   // or any of the 4 tables
        uint32_t raw = Adafruit_NeoPixel::ColorHSV(band3.hue, band3.sat, band3.val);
        raw = Adafruit_NeoPixel::gamma32(raw);
        bead.pixels.setPixelColor(i, raw);
    }
    for (int j = 0; j < 3; j++) {
        const HSVBand &band4 = greenYellowBands[j];   // or any of the 4 tables
        uint32_t raw = Adafruit_NeoPixel::ColorHSV(band4.hue, band4.sat, band4.val);
        raw = Adafruit_NeoPixel::gamma32(raw);
        bead.pixels.setPixelColor(6-j, raw);
    }
    // Do the shared middle one
    const HSVBand &band5 = redYellowBands[4];
    uint32_t shared_middle = Adafruit_NeoPixel::ColorHSV(band5.hue, band5.sat, band5.val);
    shared_middle = Adafruit_NeoPixel::gamma32(shared_middle);
    bead.pixels.setPixelColor(3, shared_middle);
    bead.pixels.show();
    delay(3000);
    bead.clear();
    bead.pixels.show();
    delay(1000);
}

// USE THIS FOR SINGLE PARAMETER SWEEPS
//void loop() {
//    uint16_t testHue = hueCandidates[hueStep];
//    //uint32_t raw = Adafruit_NeoPixel::ColorHSV(FIXED_HUE, testSat, FIXED_VAL);
//    uint32_t raw = Adafruit_NeoPixel::ColorHSV(testHue, FIXED_SAT, FIXED_VAL);
//    
//    uint8_t r = (raw >> 16) & 0xFF;
//    uint8_t g = (raw >> 8) & 0xFF;
//    uint8_t b = raw & 0xFF;
//
//    // Single pixel only (idx0) - to test cross-talk, temporarily change
//    // this to a for-loop over all 5 indices and compare by eye.
//    //bead.pixels.setPixelColor(0, raw);
//    for (int i = 0; i < 5; i++) {
//        bead.pixels.setPixelColor(i, raw);
//    }
//
//    bead.pixels.show();
//
//    Serial.print("hueStep="); Serial.print(hueStep);
//    Serial.print(" hue=");    Serial.print(testHue);
//    Serial.print("  R=");     Serial.print(r);
//    Serial.print(" G=");      Serial.print(g);
//    Serial.print(" B=");      Serial.println(b);
//
//    delay(3000);
//
//    hueStep++;
//    if (hueStep >= NUM_HUE) {
//        hueStep = 0;
//    }
//}

// USE THIS FOR MULTI-PARAMETER GAMUT SWEEPING
//void loop()
//{
//    //delay(3000);
//    //gammaToggle = gammaToggle ? false : true;
//
//    //// Color finetuning
//    //uint8_t testVal = valCandidates[valStep];
//    //uint8_t testSat = satCandidates[satStep];
//    ////bead.clear();
//    //for (int i = 4; i >= 0; i--) {
//    //    HSVBand b = redBandsHSV[i];   // copy so the source table stays untouched
////
//    //    // Only override val on the low, hard-to-see bands (idx0-idx2).
//    //    // idx3/idx4 keep their original val as a fixed bright anchor.
//    //    if (i <= 2) {
//    //        b.val = testVal;
//    //    }
//    //    b.sat = testSat;
////
//    //    uint32_t raw = Adafruit_NeoPixel::ColorHSV(b.hue, b.sat, b.val);
//    //    uint32_t corrected = 0;
//    //    if (gammaToggle)
//    //    {
//    //        uint8_t r = gammaLUT[(raw >> 16) & 0xFF];
//    //        uint8_t g = gammaLUT[(raw >> 8) & 0xFF];
//    //        uint8_t bch = gammaLUT[raw & 0xFF];
//    //        corrected = ((uint32_t)r << 16) | ((uint32_t)g << 8) | bch;
//    //    }
//    //    else 
//    //    {
//    //        corrected = raw;
//    //    }
////
////
//    //    bead.pixels.setPixelColor(i, corrected);
//    //}
//    //bead.pixels.show();
////
//    //// Print what's currently on the sphere so you can log the combo that
//    //// looked best without having to guess from memory afterwards.
//    //Serial.print("valStep=");   Serial.print(valStep);
//    //Serial.print(" testVal=");  Serial.print(testVal);
//    //Serial.print("  satStep="); Serial.print(satStep);
//    //Serial.print(" testSat=");  Serial.println(testSat);
////
//    //delay(3500);
//    //// --- Advance counters: sat cycles fully before val advances ---
//    //satStep++;
//    //if (satStep >= NUM_SAT) {
//    //    satStep = 0;
//    //    valStep++;
//    //    if (valStep >= NUM_VAL) {
//    //        valStep = 0;   // wrap around and repeat the whole grid
//    //    }
//    //}
//}

// -----------------------------------------------------------------------
// Cross-talk comparison test:
// Swap the single setPixelColor(0, raw) call above for:
//
//   for (int i = 0; i < 5; i++) {
//       bead.pixels.setPixelColor(i, raw);
//   }
//
// Run both versions back to back at the same satStep value. If the
// all-pixels version looks noticeably whiter/flatter than the single-pixel
// version at the same saturation, the diffuser is mixing light across
// pixels and adding its own desaturation on top of what you're setting
// in software - that's a shell/placement issue, not an HSV math issue.
// -----------------------------------------------------------------------
