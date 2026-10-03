#include <Qbead.h>
#include <internal/QbeadUtils.h>

using namespace Qbead;

Qbead::Qbead bead;

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

void resetBead()
{
    bead.clearStates();
    bead.clear();
    bead.show();
}

void measure()
{
    Serial.println("Tapped! Measuring...");
    bead.ble.sendData(
                BLEManager::CommandType::AddState, //TODO: add a CommandType for Measure
                0, 0, 0);
}

void collapse()
{
    Serial.println("Received tap command! Measuring the other QBead...");
}

// ---------------------------------------------

void setup()
{
    bead.begin();
    bead.setBrightness(25);
    initPixelLUT(bead);
    bead.testPixels();
    resetBead();

    bead.applyPreparedState(1);
    bead.displayCurrentStatesStatic();
}

void loop()
{
    bead.readIMU(false);
    if (bead.wasTapped())
    {
        measure();
    }

    BLEManager::DataPacket packet = bead.takeLatestPacket();
    if (packet.type == BLEManager::CommandType::AddState) // TODO: change this to a new CommandType for Measure
    {
        collapse();
    }
}