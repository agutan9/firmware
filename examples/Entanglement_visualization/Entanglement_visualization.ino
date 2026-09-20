#include <Qbead.h>
#include <QbeadUtils.h>

using namespace Qbead;

Qbead::Qbead bead;
int c_tap = 0;
int setVisual = 0;

#define NORTH_POLE_IDX 0
#define SOUTH_POLE_IDX 6
#define NUM_PIXELS 62

struct PixelAxis
{
    float x;
    float y;
    float z;
};

PixelAxis pixelLUT[NUM_PIXELS];

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
        bead.setBloch_deg(up, blue);
        bead.show();
        delay(400);
        bead.clear();
        bead.setBloch_deg(down, red);
        bead.show();
        delay(400);
    }
    if(stateNumber == 3)
    { 

    }
    if(stateNumber == 4)
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
    if(bead.wasTapped())
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
        if(c_tap > 23)
        {
            c_tap = 0;
            bead.ble.sendData(BLEManager::CommandType::ClearStates, 0, 0, 0);
            resetBead();
            setVisual = 0;
        }
        else if (c_tap >= 18 && setVisual < 4)
        {
            setVisual = 4;
            Serial.println("Setting entanglement with split-QBEADS");
            resetBead();
            bead.ble.sendData(BLEManager::CommandType::PreparedVisualizations, 4, 0, 0);
            loadPreparedVisuals(4);
        }
        else if (c_tap >= 13 && setVisual < 3)
        {
            setVisual = 3;
            Serial.println("Setting entanglement with QBEADS");
            resetBead();
            bead.ble.sendData(BLEManager::CommandType::PreparedVisualizations, 3, 0, 0);
            loadPreparedVisuals(3);            
        }
        else if (c_tap >= 10 && setVisual < 3)
        {
            setVisual = 2;
            Serial.println("Setting entanglement with cycling");
            resetBead();
            bead.ble.sendData(BLEManager::CommandType::PreparedVisualizations, 2, 0, 0);
            loadPreparedVisuals(2);
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