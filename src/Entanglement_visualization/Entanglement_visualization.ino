#include <Qbead.h>
#include <internal/QbeadUtils.h>

using namespace Qbead;

Qbead::Qbead bead;

int c_tap = 0;
uint8_t setVisual = 0;

uint32_t lastContourUpdate = 0;
bool stateSwap = false;

#define NORTH_POLE_IDX 0
#define SOUTH_POLE_IDX 6
#define NUM_PIXELS 62

struct PixelAxis
{
    float x;
    float y;
    float z;
};

struct HSVBand
{
    uint16_t hue;
    uint8_t sat;
    uint8_t val;
};

static constexpr HSVBand bandsRedYellow[5] = {
    {0, 255, 255},    // idx0: near-black red
    {1820, 255, 210}, // idx1: 10 deg
    {4005, 255, 150}, // idx2: 22 deg
    {6554, 255, 90},  // idx3: 36 deg  (your liked hue)
    {9102, 255, 40},  // idx4: 50 deg, bright yellow-orange
};
static constexpr HSVBand bandsGreenYellow[5] = {
    {21845, 255, 255}, // idx0: near-black green, 120 deg
    {18204, 255, 190}, // idx1: 100 deg (near your liked 19960)
    {15474, 255, 120}, // idx2: 85 deg
    {11833, 255, 70},  // idx3: 65 deg
    {9102, 255, 40},   // idx4: 50 deg, bright yellow (shared anchor w/ red map)
};
static constexpr HSVBand bandsBlueYellow[5] = {
    {39321, 255, 200},
    {32178, 255, 80},
    {25036, 255, 40},
    {17893, 255, 80},
    {10751, 255, 200},
};
static bool gammaCorrect = true;

PixelAxis pixelLUT[NUM_PIXELS];

void resetBead();
void loadPreparedVisuals(uint8_t visual);
void activateVisual(uint8_t visual);
void updateVisual();

uint32_t pauliColorMap(float geomInProd)
{
    const float mag = fabsf(geomInProd);

    uint8_t idx;

    if (mag < 0.05f)
    { // ~0.0 (orthogonal to axis)
        idx = 0;
    }
    else if (mag < 0.25f)
    { // ~0.13 (small angle off orthognal)
        idx = 1;
    }
    else if (mag < 0.70f)
    { // ~0.50 (45deg)
        idx = 2;
    }
    else if (mag < 0.95f)
    { // ~0.87 (smal angle off parallel)
        idx = 3;
    }
    else
    { // ~1.00 (parallel to axis)
        idx = 4;
    }

    const uint8_t revIdx = 4 - idx;

    const HSVBand bandHSV =
        (geomInProd >= 0.0f)
            ? bandsRedYellow[revIdx]
            : bandsGreenYellow[revIdx];

    return Adafruit_NeoPixel::ColorHSV(
        bandHSV.hue,
        bandHSV.sat,
        bandHSV.val);
}

uint32_t entanglementColorMap(float geomInProd)
{
    const float mag = fabsf(geomInProd);

    uint8_t idx;
    // hardcoded to be 90deg offset from tracking axis
    if (mag < 0.05f)
    { // ~1.00 (parallel to axis)
        idx = 4;
    }
    else if (mag < 0.25f)
    { // ~0.13 (small angle off orthognal)
        idx = 3;
    }
    else if (mag < 0.70f)
    { // ~0.50 (45deg)
        idx = 2;
    }
    else if (mag < 0.95f)
    { // ~0.87 (smal angle off parallel)
        idx = 1;
    }
    else
    { // ~1.0 (orthogonal to axis)
        idx = 0;
    }

    const HSVBand bandHSV = bandsBlueYellow[idx];

    return Adafruit_NeoPixel::ColorHSV(
        bandHSV.hue,
        bandHSV.sat,
        bandHSV.val);
}

void showContourBands(const Qbead::BlochVector &arbAxis, bool shared)
{
    bead.clear();

    for (int pixelIndex = 0; pixelIndex < NUM_PIXELS; pixelIndex++)
    {
        const float dotProduct =
            pixelLUT[pixelIndex].x * arbAxis.x +
            pixelLUT[pixelIndex].y * arbAxis.y +
            pixelLUT[pixelIndex].z * arbAxis.z;

        uint32_t pixelColour;

        if (shared)
        {
            pixelColour = entanglementColorMap(dotProduct);
        }
        else
        {
            pixelColour = pauliColorMap(dotProduct);
        }

        if (gammaCorrect)
        {
            pixelColour = Adafruit_NeoPixel::gamma32(pixelColour);
        }

        bead.pixels.setPixelColor(pixelIndex, pixelColour);
    }

    bead.show();
}

void showYellowBlackHalves()
{
    const uint32_t yellow = color(255, 255, 0);
    const uint32_t black = color(30, 30, 30);

    bead.clear();

    for (int pixelIndex = 0; pixelIndex < NUM_PIXELS; pixelIndex++)
    {
        if (pixelLUT[pixelIndex].z >= 0.0f)
        {
            bead.pixels.setPixelColor(pixelIndex, yellow);
        }
        else
        {
            bead.pixels.setPixelColor(pixelIndex, black);
        }
    }

    bead.show();
}

void resetBead()
{
    bead.clearStates();
    bead.clear();
    bead.show();
}

void loadPreparedVisuals(uint8_t visual)
{
    const BlochVector up(0, 0);

    switch (visual)
    {
    case 1:
    {
        bead.applyPreparedState(1);
        bead.displayCurrentStatesStatic();
        break;
    }

    case 2:
    {
        bead.applyPreparedState(1);

        bead.cyclingIndex = 0;
        bead.lastChange = 0;

        // Shows the first state immediately.
        bead.displayCurrentStatesCycling();
        break;
    }

    case 3:
    {
        stateSwap = false;
        lastContourUpdate = 0;

        // First frame: shared/entanglement colour map.
        showContourBands(up, true);
        break;
    }

    case 4:
    {
        showYellowBlackHalves();
        break;
    }

    default:
    {
        break;
    }
    }
}

void activateVisual(uint8_t visual)
{
    if (setVisual == visual)
    {
        return;
    }

    setVisual = visual;

    resetBead();

    loadPreparedVisuals(setVisual);
}

void updateVisual()
{
    if (setVisual == 2)
    {
        bead.displayCurrentStatesCycling();
        return;
    }

    if (setVisual == 3)
    {
        const uint32_t now = millis();

        if (now - lastContourUpdate < 400)
        {
            return;
        }

        lastContourUpdate = now;

        const BlochVector up(0, 0);

        stateSwap = !stateSwap;

        showContourBands(up, stateSwap);
    }
}

void initPixelLUT(const Qbead::Qbead &bead)
{
    pixelLUT[NORTH_POLE_IDX] = {0.0f, 0.0f, -1.0f};
    pixelLUT[SOUTH_POLE_IDX] = {0.0f, 0.0f, 1.0f};

    const int pixelsPerLeg = bead.nsections - 1;
    // First physical leg: pixel order runs from south toward north
    for (int thetaIndex = 1; thetaIndex < bead.nsections; thetaIndex++)
    {
        const float theta =
            180.0f - thetaIndex * bead.theta_quant;

        pixelLUT[thetaIndex] = {
            Qbead::sin_deg(theta),
            0.0f,
            Qbead::cos_deg(theta)};
    }
    // Remaining physical legs: theta runs from north toward south
    for (int phiIndex = 1; phiIndex < bead.nlegs; phiIndex++)
    {
        const float phi = phiIndex * bead.phi_quant;

        for (int thetaIndex = 1; thetaIndex < bead.nsections; thetaIndex++)
        {
            const float theta = thetaIndex * bead.theta_quant;

            const int pixelIndex =
                7 +
                (phiIndex - 1) * pixelsPerLeg +
                (thetaIndex - 1);

            const float sinTheta = Qbead::sin_deg(theta);

            pixelLUT[pixelIndex] = {
                Qbead::cos_deg(phi) * sinTheta,
                Qbead::sin_deg(phi) * sinTheta,
                Qbead::cos_deg(theta)};
        }
    }
}

void setup()
{
    bead.begin();
    bead.setBrightness(25);
    initPixelLUT(bead);
    bead.testPixels();
    resetBead();
}

void loop()
{
    bead.readIMU(false);
    if (bead.wasTapped())
    {
        c_tap++;
        Serial.print("Tap count: ");
        Serial.println(c_tap);
    }

    BLEManager::DataPacket packet = bead.takeLatestPacket();
    if (packet.type == BLEManager::CommandType::PreparedVisualizations)
    {
        Serial.print("Received visual: ");
        Serial.println(packet.value);
        activateVisual(packet.value);
    }
    else if (packet.type == BLEManager::CommandType::ClearStates)
    {
        Serial.println("Received clear command");
        c_tap = 0;
        setVisual = 0;
        resetBead();
    }
    else
    {
        if (c_tap > 25)
        {
            c_tap = 0;
            bead.ble.sendData(
                BLEManager::CommandType::ClearStates,
                0, 0, 0);

            setVisual = 0;
            resetBead();
        }
        else if (c_tap >= 20)
        {
            if (setVisual != 4)
            {
                Serial.println("Selecting visual 4");
                activateVisual(4);
                bead.ble.sendData(
                    BLEManager::CommandType::PreparedVisualizations,
                    4, 0, 0);
            }
        }
        else if (c_tap >= 15)
        {
            if (setVisual != 3)
            {
                Serial.println("Selecting visual 3");
                activateVisual(3);
                bead.ble.sendData(
                    BLEManager::CommandType::PreparedVisualizations,
                    3, 0, 0);
            }
        }
        else if (c_tap >= 10)
        {
            if (setVisual != 2)
            {
                Serial.println("Selecting visual 2");
                activateVisual(2);
                bead.ble.sendData(
                    BLEManager::CommandType::PreparedVisualizations,
                    2, 0, 0);
            }
        }
        else if (c_tap >= 5)
        {
            if (setVisual != 1)
            {
                Serial.println("Selecting visual 1");
                activateVisual(1);
                bead.ble.sendData(
                    BLEManager::CommandType::PreparedVisualizations,
                    1, 0, 0);
            }
        }
    }

    // Animated modes are advanced here, once per main-loop pass
    updateVisual();
}