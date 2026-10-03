#include <Qbead.h>
#include <internal/QbeadUtils.h>

using namespace Qbead;

Qbead::Qbead bead;

#define NORTH_POLE_IDX 0
#define SOUTH_POLE_IDX 6
#define NUM_PIXELS 62
static constexpr uint32_t ENTANGLED_STATE = 1;

// How long to keep the entangled-state display before clearing it.
static constexpr uint32_t ENTANGLED_DISPLAY_MS = 15000;

bool entangledDisplayActive = false;
uint32_t entangledAtMs = 0;

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
    const uint32_t outcome = random(2); // 0 or 1

    Serial.print("Tapped! Measuring outcome |");
    Serial.print(outcome);
    Serial.println(">");

    bead.ble.sendData(
        BLEManager::CommandType::Measure,
        outcome,
        0,
        0);

    setCollapsedState(outcome);
    entangledDisplayActive = false;
}
void setCollapsedState(uint32_t outcome)
{
    bead.clearStates();

    BlochVector up(0, 0);
    BlochVector down(180, 0);

    if (outcome == 1)
    {
        bead.addState(up);
    }
    else
    {
        bead.addState(down);
    }

    bead.displayCurrentStatesStatic();
}

void collapse(uint32_t remoteOutcome)
{
    Serial.print("[MEASURE] Remote outcome: |");
    Serial.print(remoteOutcome);
    Serial.print(">, local outcome: |");
    Serial.print(remoteOutcome);
    Serial.println(">");

    setCollapsedState(remoteOutcome);

    entangledDisplayActive = false;
}

void showIdle()
{
    // A dim purple marker at the north pole means "ready".
    bead.clear();
    bead.setBloch_deg(0, 0, color(15, 0, 15));
    bead.show();
}

void showLocalRequestPending()
{
    bead.clear();
    bead.setBloch_deg(0, 0, color(0, 0, 80));
    bead.show();
}

void showRemoteRequestPending()
{
    bead.clear();
    bead.setBloch_deg(180, 0, color(80, 0, 80));
    bead.show();
}

void showEntanglementSuccess()
{
    bead.displayCurrentStatesStatic();
}

void showTimeout()
{
    bead.clear();
    bead.setBloch_deg(90, 0, color(80, 0, 0));
    bead.show();
}

void setup()
{
    bead.begin();
    bead.setBrightness(25);
    initPixelLUT(bead);
    bead.testPixels();
    resetBead();
    showIdle();
}

void loop()
{
    bead.readIMU(false);

    // First handle a measurement initiated by the other Qbead.
    BLEManager::DataPacket packet = bead.takeLatestPacket();

    if (packet.type == BLEManager::CommandType::Measure &&
        entangledDisplayActive)
    {
        collapse(packet.value);
    }

    if (!entangledDisplayActive)
    {
        // Taps are interpreted as entanglement requests here.
        if (bead.entangle(ENTANGLED_STATE))
        {
            entangledDisplayActive = true;
            entangledAtMs = millis();

            showEntanglementSuccess();

            Serial.println("[SUCCESS] Entangled.");
        }
    }
    else
    {
        // Taps are interpreted as measurements here.
        if (bead.wasTapped())
        {
            measure();
        }
    }

    if (entangledDisplayActive &&
        (uint32_t)(millis() - entangledAtMs) >= ENTANGLED_DISPLAY_MS)
    {
        entangledDisplayActive = false;
        bead.clearStates();
        showIdle();

        Serial.println("[INFO] Entangled state timed out.");
    }
}