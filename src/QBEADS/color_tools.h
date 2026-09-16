// color_tools.h
// -----------------------------------------------------------------------
// Bipolar hemisphere color-band helpers for the Qbead Bloch-sphere LED shell.
// Provides:
//   1. colorHSL()          - HSL->RGB packer (S=100% oriented, matches your
//                             existing redBands/greenBands derivation)
//   2. colorHSV_custom()   - Wrapper around Adafruit_NeoPixel::ColorHSV with
//                             the same (hue, level) call signature as colorHSL()
//   3. gammaCorrect()      - Tunable single-channel gamma function, drop-in
//                             replacement/extension of strip.gamma8()/gamma32()
//   4. redBandsHSV / greenBandsHSV - starting HSV-native band tables that are
//                             numerically equivalent to your existing hex bands
// -----------------------------------------------------------------------

#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// -----------------------------------------------------------------------
// 0. WHY THE PORT IS EXACT
// -----------------------------------------------------------------------
// Your bands were built as HSL with S=100%. At S=100%, HSL forces
// min(R,G,B)=0, and it turns out max(R,G,B) in your table already equals
// what HSV would call V (0-255):
//
//   redBands   max-channel: 31, 82, 143, 204, 255
//   greenBands max-channel: 31, 82, 143, 204, 255
//
// So the "same" ramp in HSV space is just: keep Sat=255 (100%), keep the
// same Hue angle per band, and set Val = that same byte sequence.
// That's exactly what redBandsHSV / greenBandsHSV below encode.
// -----------------------------------------------------------------------

// ===========================================================================
// 1. HSL METHOD  (float-friendly, returns packed 0x00RRGGBB Color)
// ===========================================================================
// hueDeg   : 0-360 (wraps automatically)
// satPct   : 0-100
// lightPct : 0-100
// This mirrors classic HSL math (the same model you used to author the
// original bands) so you can keep authoring new bands the way you already
// think about them, or A/B them directly against the HSV version below.
static inline uint32_t colorHSL(float hueDeg, float satPct, float lightPct) {
    float h = fmodf(hueDeg, 360.0f);
    if (h < 0) h += 360.0f;
    float s = constrain(satPct, 0.0f, 100.0f) / 100.0f;
    float l = constrain(lightPct, 0.0f, 100.0f) / 100.0f;

    float c = (1.0f - fabsf(2.0f * l - 1.0f)) * s;
    float hp = h / 60.0f;
    float x = c * (1.0f - fabsf(fmodf(hp, 2.0f) - 1.0f));
    float m = l - c / 2.0f;

    float r1, g1, b1;
    if (hp < 1)      { r1 = c; g1 = x; b1 = 0; }
    else if (hp < 2) { r1 = x; g1 = c; b1 = 0; }
    else if (hp < 3) { r1 = 0; g1 = c; b1 = x; }
    else if (hp < 4) { r1 = 0; g1 = x; b1 = c; }
    else if (hp < 5) { r1 = x; g1 = 0; b1 = c; }
    else             { r1 = c; g1 = 0; b1 = x; }

    uint8_t r = (uint8_t)roundf((r1 + m) * 255.0f);
    uint8_t g = (uint8_t)roundf((g1 + m) * 255.0f);
    uint8_t b = (uint8_t)roundf((b1 + m) * 255.0f);

    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

// ===========================================================================
// 2. HSV METHOD  (same call shape, backed by NeoPixel::ColorHSV)
// ===========================================================================
// hueDeg  : 0-360 (float, for readability while tuning; converted to the
//           16-bit hue NeoPixel expects internally)
// satPct  : 0-100
// valPct  : 0-100  <-- this replaces "lightness"; at S=100% this is the
//                      direct analogue of your original band ramps
//
// Use this one for the actual firmware; ColorHSV() is implemented with
// integer math on the library side and is what gamma32() is calibrated
// against, so pairing them gives you the "intended" NeoPixel pipeline.
static inline uint32_t colorHSV_custom(float hueDeg, float satPct, float valPct) {
    float h = fmodf(hueDeg, 360.0f);
    if (h < 0) h += 360.0f;

    uint16_t hue16 = (uint16_t)roundf(h / 360.0f * 65535.0f);
    uint8_t  sat8  = (uint8_t)roundf(constrain(satPct, 0.0f, 100.0f) / 100.0f * 255.0f);
    uint8_t  val8  = (uint8_t)roundf(constrain(valPct, 0.0f, 100.0f) / 100.0f * 255.0f);

    // Adafruit_NeoPixel::ColorHSV is a static member; no instance needed.
    return Adafruit_NeoPixel::ColorHSV(hue16, sat8, val8);
}

// ===========================================================================
// 3. TUNABLE GAMMA CORRECTION
// ===========================================================================
// Generalizes strip.gamma8()/gamma32(): NeoPixel bakes in a fixed gamma of
// 2.8 with hardcoded max_in/max_out=255. Here every parameter is exposed so
// you can retune per-channel or per-shell without touching a lookup table.
//
//   gammaCorrect8(x, gamma, maxIn, maxOut)  - single byte, direct formula
//   gammaCorrectColor(color, gamma, ...)    - applies it to all 3 channels
//                                              of a packed 0x00RRGGBB color
//
// Typical starting points for a diffuse acrylic/PLA shell that's washing
// out contrast between bands: try gamma in the 2.2-3.0 range (2.8 is the
// NeoPixel default and a good CRT-perceptual baseline); raise it toward
// ~3.5-4.0 if the low bands still look too bright/washed out, since a
// higher exponent pushes more of the low range toward black.
static inline uint8_t gammaCorrect8(uint8_t x,
                                     float gamma = 2.8f,
                                     uint8_t maxIn = 255,
                                     uint8_t maxOut = 255) {
    if (maxIn == 0) return 0;
    float normalized = (float)x / (float)maxIn;
    float corrected = powf(normalized, gamma);
    long out = lroundf(corrected * (float)maxOut);
    if (out < 0) out = 0;
    if (out > 255) out = 255;
    return (uint8_t)out;
}

// Applies gammaCorrect8 independently to R, G, B (ignores/pass-through W if
// you're on RGBW hardware and pack it in the top byte).
static inline uint32_t gammaCorrectColor(uint32_t color,
                                          float gamma = 2.8f,
                                          uint8_t maxIn = 255,
                                          uint8_t maxOut = 255) {
    uint8_t w = (uint8_t)(color >> 24);
    uint8_t r = (uint8_t)(color >> 16);
    uint8_t g = (uint8_t)(color >> 8);
    uint8_t b = (uint8_t)(color);

    r = gammaCorrect8(r, gamma, maxIn, maxOut);
    g = gammaCorrect8(g, gamma, maxIn, maxOut);
    b = gammaCorrect8(b, gamma, maxIn, maxOut);

    return ((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

// Optional: precompute a full 0-255 gamma LUT once at startup if you want
// gamma8()-style O(1) lookups instead of calling powf() per pixel per frame.
// Call buildGammaTable(myTable, 3.2f) then read myTable[x] like gamma8[x].
static inline void buildGammaTable(uint8_t table[256],
                                    float gamma = 2.8f,
                                    uint8_t maxIn = 255,
                                    uint8_t maxOut = 255) {
    for (int i = 0; i <= 255; i++) {
        table[i] = gammaCorrect8((uint8_t)i, gamma, maxIn, maxOut);
    }
}

// ===========================================================================
// 4. STARTING HSV-NATIVE BAND TABLES
// ===========================================================================
// Numerically equivalent starting point to your existing hex bands, in
// (hue16, sat8, val8) form so you can feed them straight into
// Adafruit_NeoPixel::ColorHSV() or colorHSV_custom() and then hand-tune V
// (and optionally hue/sat) per band to fight the diffuser wash-out.
//
// idx0: near-black blue -> idx4: pure red/green, same hue anchors as before.
struct HSVBand { uint16_t hue; uint8_t sat; uint8_t val; };

static constexpr HSVBand redBandsHSV[5] = {
    {43690, 255, 31},   // hue 240 deg (blue)   - idx0
    {51810, 255, 82},   // hue ~284.6 deg       - idx1
    {57361, 255, 143},  // hue ~315.1 deg       - idx2
    {62805, 255, 204},  // hue 345 deg          - idx3
    {0,     255, 255},  // hue 0 deg (red)      - idx4
};

static constexpr HSVBand greenBandsHSV[5] = {
    {43690, 255, 31},   // hue 240 deg (blue)   - idx0 (shared anchor with red)
    {38229, 255, 82},   // hue 210 deg          - idx1
    {32768, 255, 143},  // hue 180 deg          - idx2
    {27306, 255, 204},  // hue 150 deg          - idx3
    {21845, 255, 255},  // hue 120 deg (green)  - idx4
};

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
    {9102,  255, 40},   // idx4: 50 deg, bright yellow-orange
};

// ===========================================================================
// GREEN -> YELLOW -> BLACK
// Other bipolar-hemisphere pole. idx0 = dark green anchor, idx4 shares the
// same yellow endpoint as redYellowBands (intentional - see note below).
// hue16=18204 (100 deg) at idx1 is close to your liked 19960 (109.6 deg).
// ===========================================================================
static constexpr HSVBand greenYellowBands[5] = {
    {21845, 255, 255},   // idx0: near-black green, 120 deg
    {18204, 255, 190},   // idx1: 100 deg (near your liked 19960)
    {15474, 255, 120},  // idx2: 85 deg
    {11833, 255, 70},  // idx3: 65 deg
    {9102,  255, 40},  // idx4: 50 deg, bright yellow (shared anchor w/ red map)
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
// CORRELATION BEADS - now YELLOW (#fffb00), replacing the blue-violet option
// idx0 = dark yellow anchor, idx4 = full #fffb00 at max brightness.
// ===========================================================================
static constexpr HSVBand correlationBlueYellowBands[5] = {
    {39321, 255, 200},   // idx0: dark #fffb00-hue anchor
    {32178, 255, 80},  // idx1
    {25036, 255, 40},  // idx2
    {17893, 255, 80},  // idx3
    {10751, 255, 200},  // idx4: full #fffb00 bright anchor
};

// Example usage for tuning:
//
//   uint8_t gammaLUT[256];
//   buildGammaTable(gammaLUT, 3.2f);   // start steeper than default 2.8
//
//   for (int i = 0; i < 5; i++) {
//       uint32_t raw = Adafruit_NeoPixel::ColorHSV(
//           redBandsHSV[i].hue, redBandsHSV[i].sat, redBandsHSV[i].val);
//       uint8_t r = gammaLUT[(raw >> 16) & 0xFF];
//       uint8_t g = gammaLUT[(raw >> 8) & 0xFF];
//       uint8_t b = gammaLUT[raw & 0xFF];
//       uint32_t corrected = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
//       strip.setPixelColor(i, corrected);
//   }
//
// To push more contrast against the diffuse shell without touching hue/sat:
//   - Lower val on idx0-idx2 further (e.g. 20, 60, 120) to deepen the dark end.
//   - Raise gamma (try 3.0-4.0) so mid-tones get pulled down harder than highs.
//   - Keep idx4 pinned near val=255 as your fixed "full saturation" anchor.
