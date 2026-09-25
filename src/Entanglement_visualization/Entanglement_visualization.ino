#include <Qbead.h>
#include <internal/QbeadUtils.h>

using namespace Qbead;

Qbead::Qbead bead;
int c_tap = 0;
int setVisual = 0;
uint32_t lastTick = 0;
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

uint32_t pauliColorMap(float geomInProd)
{

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
    uint8_t rev_idx = 4 - idx;
    HSVBand band_HSV = (geomInProd >= 0.0f) ? bandsRedYellow[rev_idx] : bandsGreenYellow[rev_idx];
    return Adafruit_NeoPixel::ColorHSV(band_HSV.hue, band_HSV.sat, band_HSV.val);
}

uint32_t entanglementColorMap(float geomInProd)
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
    HSVBand band_HSV = bandsBlueYellow[idx];
    return Adafruit_NeoPixel::ColorHSV(band_HSV.hue, band_HSV.sat, band_HSV.val);
}

void showContourBands(const Qbead::BlochVector &arbAxis, bool shared)
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float sum = 0.0f;

    bead.clear();

    for (int p_i = 0; p_i < NUM_PIXELS; p_i++)
    {
        // in-product with pixel's basis unit vecs
        x = pixelLUT[p_i].x * arbAxis.x;
        y = pixelLUT[p_i].y * arbAxis.y;
        z = pixelLUT[p_i].z * arbAxis.z;
        sum = x + y + z;
        uint32_t color = 0;
        if (shared)
        {
            color = entanglementColorMap(sum);
        }
        else
        {
            color = pauliColorMap(sum);
        }
        color = gammaCorrect ? Adafruit_NeoPixel::gamma32(color) : color;

        bead.pixels.setPixelColor(p_i, color);
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

void loadPreparedVisuals(uint8_t stateNumber)
{
    BlochVector up(0, 0);
    BlochVector down(180, 0);
    BlochVector plusState(90, 0);
    uint32_t blue = color(0, 0, 255);
    uint32_t red = color(255, 0, 0);

    if (stateNumber == 1)
    {
        bead.addState(up);
        bead.addState(down);
        bead.setBloch_deg(up, blue);
        bead.setBloch_deg(down, red);
        bead.show();
    }
    if (stateNumber == 2)
    {
        if (stateSwap)
        {
            bead.setBloch_deg(up, blue);
        }
        else
        {
            bead.setBloch_deg(down, red);
        }
        stateSwap = !stateSwap;
        bead.show();
    }
    if (stateNumber == 3)
    {
        if (stateSwap)
        {
            showContourBands(up, false);
        }
        else
        {
            showContourBands(up, true);
        }
        stateSwap = !stateSwap;
    }
    if (stateNumber == 4)
    {
        Serial.println("Showing yellow/black split sphere");
        showYellowBlackHalves();
    }
}

void initPixelLUT(const Qbead::Qbead &bead)
{
    // Current physical Qbead pixel/index convention.
    pixelLUT[NORTH_POLE_IDX] = {0.0f, 0.0f, -1.0f};
    pixelLUT[SOUTH_POLE_IDX] = {0.0f, 0.0f, 1.0f};

    const int pixelsPerLeg = bead.nsections - 1;

    // First physical leg: pixel order runs from south toward north.
    for (int thetaIndex = 1; thetaIndex < bead.nsections; thetaIndex++)
    {
        float theta = 180.0f - thetaIndex * bead.theta_quant;

        pixelLUT[thetaIndex] = {
            Qbead::sin_deg(theta),
            0.0f,
            Qbead::cos_deg(theta)};
    }

    // Remaining physical legs: theta runs from north toward south.
    for (int phiIndex = 1; phiIndex < bead.nlegs; phiIndex++)
    {
        float phi = phiIndex * bead.phi_quant;

        for (int thetaIndex = 1; thetaIndex < bead.nsections; thetaIndex++)
        {
            float theta = thetaIndex * bead.theta_quant;

            int pixelIndex =
                7 +
                (phiIndex - 1) * pixelsPerLeg +
                (thetaIndex - 1);

            float sinTheta = Qbead::sin_deg(theta);

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

void resetBead()
{
    bead.innerStateCount = 0;
    bead.clear();
    bead.show();
}

void loop()
{
    bead.readIMU(false);
    if (bead.wasTapped())
    {
        c_tap++;
        Serial.println(c_tap);
    }

    BLEManager::DataPacket packet = bead.takeLatestPacket();
    if (packet.type == BLEManager::CommandType::PreparedVisualizations)
    {
        Serial.println("state update received");
        resetBead();
        loadPreparedVisuals(packet.value);
    }
    else if (packet.type == BLEManager::CommandType::ClearStates)
    {
        Serial.println("clear command received");
        bead.innerStateCount = 0;
        resetBead();
    }
    else
    {
        if (c_tap > 25)
        {
            c_tap = 0;
            bead.ble.sendData(BLEManager::CommandType::ClearStates, 0, 0, 0);
            resetBead();
            setVisual = 0;
        }
        else if (c_tap >= 20 && setVisual < 4)
        {
            setVisual = 4;
            Serial.println("Setting entanglement with split-QBEADS");
            resetBead();
            bead.ble.sendData(BLEManager::CommandType::PreparedVisualizations, 4, 0, 0);
            loadPreparedVisuals(4);
        }
        else if (c_tap >= 15 && setVisual < 4)
        {
            uint32_t current = millis();
            uint32_t deltaTime = current - lastTick;

            if (deltaTime >= 400 || setVisual < 3)
            {
                lastTick = current;
                setVisual = 3;
                Serial.println("Setting entanglement with QBEADS");
                resetBead();
                bead.ble.sendData(BLEManager::CommandType::PreparedVisualizations, 3, 0, 0);
                loadPreparedVisuals(3);
            }
        }
        else if (c_tap >= 10 && setVisual < 3)
        {
            uint32_t current = millis();
            uint32_t deltaTime = current - lastTick;

            if (deltaTime >= 400 || setVisual < 2)
            {
                lastTick = current;
                setVisual = 2;
                Serial.println("Setting entanglement with cycling");
                resetBead();
                bead.ble.sendData(BLEManager::CommandType::PreparedVisualizations, 2, 0, 0);
                loadPreparedVisuals(2);
            }
        }
        else if (c_tap >= 5 && setVisual < 1)
        {
            setVisual = 1;
            Serial.println("Setting entanglement with colours");
            resetBead();
            loadPreparedVisuals(1);
            bead.ble.sendData(BLEManager::CommandType::PreparedVisualizations, 1, 0, 0);
        }
    }
}
