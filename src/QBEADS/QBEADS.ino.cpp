# 1 "C:\\Users\\TimBr\\AppData\\Local\\Temp\\tmpg0ws6o2h"
#include <Arduino.h>
# 1 "C:/Users/TimBr/Study/QIST/SecondYear/Q5PROJECT/Qbead/MDP_repo/src/QBEADS/QBEADS.ino"
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>
#include <bluefruit.h>
#include <Qbead.h>

#include "color_tools.h"



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


PixelAxis pixelLUT[NUM_PIXELS];


void initPixelLUT(const Qbead::Qbead &bead);
uint32_t mapRedBlackGreenDiscontinuous(float geomInProd);
uint32_t getContourColour(float geomInProd);
void setContourBands(Qbead::Qbead &bead, const Qbead::BlochVector &arbAxis);





static const uint8_t valCandidates[] = { 15, 20, 25, 31, 45, 60, 90, 120 };
static const uint8_t NUM_VAL = sizeof(valCandidates) / sizeof(valCandidates[0]);



static const uint8_t satCandidates[] = { 255, 180, 90, 60 };
static const uint8_t NUM_SAT = sizeof(satCandidates) / sizeof(satCandidates[0]);

static uint8_t valStep = 0;
static uint8_t satStep = 0;

uint8_t gammaLUT[256];
bool gammaToggle = false;
void setup();
void loop();
uint32_t mapRedBlackGreenContinuous(uint8_t colorVal);
float inProdToColorPos(float geomInProd);
#line 54 "C:/Users/TimBr/Study/QIST/SecondYear/Q5PROJECT/Qbead/MDP_repo/src/QBEADS/QBEADS.ino"
void setup() {
    Serial.begin(9600);
    delay(50);

    bead.begin();
    bead.pixels.setBrightness(255);
    initPixelLUT(bead);
    bead.testPixels();
    delay(50);
    setContourBands(bead, state);
    bead.pixels.show();
    delay(100);

    test_rot_axis.setXYZ(0.0f, 1.0f, 0.0f);
    bead.clear();


    buildGammaTable(gammaLUT, 2.8f);
}

void loop() {
    delay(100);

    bead.clear();
# 88 "C:/Users/TimBr/Study/QIST/SecondYear/Q5PROJECT/Qbead/MDP_repo/src/QBEADS/QBEADS.ino"
    setContourBands(bead, state);
    bead.pixels.show();


    delay(3000);
    gammaToggle = gammaToggle ? false : true;
# 145 "C:/Users/TimBr/Study/QIST/SecondYear/Q5PROJECT/Qbead/MDP_repo/src/QBEADS/QBEADS.ino"
}



void initPixelLUT(const Qbead::Qbead &bead)
{
# 162 "C:/Users/TimBr/Study/QIST/SecondYear/Q5PROJECT/Qbead/MDP_repo/src/QBEADS/QBEADS.ino"
    pixelLUT[0] = {0.0, 0.0, -1.0};

    pixelLUT[6] = {0.0, 0.0, +1.0};



    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;




    float cum_theta = bead.nsections * bead.theta_quant;

    for (int fleg_th = 1; fleg_th < bead.nsections; ++fleg_th){


        cum_theta -= bead.theta_quant;
#ifdef VERBOSE
        x = Qbead::sin_deg(cum_theta);
        y = 0.0f;
        z = Qbead::cos_deg(cum_theta);
        Serial.printf("Pixel ID[%d] Coords: {X:%.2f, Y:%.2f, Z:%.2f}\n", fleg_th, x, y, z);
#endif

        pixelLUT[fleg_th] = {
            Qbead::sin_deg(cum_theta),
            0.0f,
            Qbead::cos_deg(cum_theta)
        };
    }

    float band_sin_theta = 0.0f;
    float leg_cos_phi = 0.0f;
    float leg_sin_phi = 0.0f;
    int shift = bead.nsections - 1;
    int p_i = 1;

    for (int ph_j = 1; ph_j < bead.nlegs; ++ph_j)
    {
        leg_cos_phi = Qbead::cos_deg(ph_j * bead.phi_quant);
        leg_sin_phi = Qbead::sin_deg(ph_j * bead.phi_quant);
        cum_theta = 0.0f;
        for (int th_i = 2; th_i <= bead.nsections; ++th_i)
        {
            p_i = th_i + ph_j * shift;

            cum_theta += bead.theta_quant;
            band_sin_theta = Qbead::sin_deg(cum_theta);
#ifdef VERBOSE
            x = leg_cos_phi * band_sin_theta;
            y = leg_sin_phi * band_sin_theta;
            z = Qbead::cos_deg(cum_theta);
            Serial.printf("Pixel ID[%d] Coords: {X:%.2f, Y:%.2f, Z:%.2f}\n", p_i, x, y, z);
#endif
            pixelLUT[p_i] = {
                leg_cos_phi * band_sin_theta,
                leg_sin_phi * band_sin_theta,
                Qbead::cos_deg(cum_theta)
            };
        }
    }
}


uint32_t mapRedBlackGreenDiscontinuous(float geomInProd)
{
# 245 "C:/Users/TimBr/Study/QIST/SecondYear/Q5PROJECT/Qbead/MDP_repo/src/QBEADS/QBEADS.ino"
    float mag = geomInProd;
    if (geomInProd <= 0) mag *= -1;
    int idx = 0.0f;
    if (mag < 0.05f) idx = 0;
    else if (mag < 0.25f) idx = 1;
    else if (mag < 0.7f) idx = 2;
    else if (mag < 0.95f) idx = 3;
    else idx = 4;
#ifdef VERBOSE
    Serial.printf("In Prod[%.2f] gives mag: %.2f and idx: %d\n", geomInProd, mag, idx);
#endif


    uint8_t rev_idx = 4 - idx;
    HSVBand band_HSV = (geomInProd >= 0.0f) ? redYellowBands[rev_idx] : greenYellowBands[rev_idx];
    return Adafruit_NeoPixel::ColorHSV(band_HSV.hue, band_HSV.sat, band_HSV.val);
}


uint32_t mapRedBlackGreenContinuous(uint8_t colorVal)
{

    if (colorVal < 128)
    {
        uint8_t g = (colorVal - 128) * 2;
        return Adafruit_NeoPixel::Color(0, g, 0);
    }
    else
    {
        uint8_t r = 255 - colorVal * 2;
        return Adafruit_NeoPixel::Color(r, 0, 0);
    }
}



float inProdToColorPos(float geomInProd)
{





    uint8_t colorPos = (uint8_t)(geomInProd * 127.5f + 128.0f);
}


uint32_t getContourColour(float geomInProd, bool gammaCorrect = true)
{

    uint32_t colorMapped = mapRedBlackGreenDiscontinuous(geomInProd);
    return gammaCorrect ? Adafruit_NeoPixel::gamma32(colorMapped) : colorMapped;
}

void setContourBands(Qbead::Qbead &bead, const Qbead::BlochVector &arbAxis)
{

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

            , false
        ));
    }
}
# 335 "C:/Users/TimBr/Study/QIST/SecondYear/Q5PROJECT/Qbead/MDP_repo/src/QBEADS/QBEADS.ino"
void showPixelIdsOneByOne(Qbead::Qbead &bead, uint8_t waitTime, uint8_t lastID)
{

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