#include <Arduino.h>
#include "Qbead.h"
#include "internal/QbeadUtils.h"

Qbead::Qbead qbead;

// The state passed to qbead.entangle(state).
//
// state == 1:
//   first displayed state:  |0> / north pole
//   second displayed state: |1> / south pole
//
// state == 2 reverses their order, but because your static display iterates
// through both states, the visible result is presently similar.
static constexpr uint32_t ENTANGLED_STATE = 1;

// How long to keep the entangled-state display before clearing it.
static constexpr uint32_t ENTANGLED_DISPLAY_MS = 3000;

bool entangledDisplayActive = false;
uint32_t entangledAtMs = 0;

using namespace Qbead;

void showIdle()
{
    // A dim purple marker at the north pole means "ready".
    qbead.clear();
    qbead.setBloch_deg(0, 0, color(15, 0, 15));
    qbead.show();
}

void showLocalRequestPending()
{
    // Blue north-pole marker means:
    // "I tapped and sent an Entangle request; waiting for partner."
    qbead.clear();
    qbead.setBloch_deg(0, 0, color(0, 0, 80));
    qbead.show();
}

void showRemoteRequestPending()
{
    // Magenta south-pole marker means:
    // "A partner requested entanglement; tap this Qbead now to finish handshake."
    qbead.clear();
    qbead.setBloch_deg(180, 0, color(80, 0, 80));
    qbead.show();
}

void showEntanglementSuccess()
{
    // entangle() calls applyPreparedState(), which fills qbead.innerStates.
    // This displays the two prepared Bloch-sphere states using stateColours.
    qbead.displayCurrentStatesStatic();
}

void showTimeout()
{
    // Red equatorial marker means a request expired.
    qbead.clear();
    qbead.setBloch_deg(90, 0, color(80, 0, 0));
    qbead.show();
}

void setup()
{
    qbead.begin();

    qbead.setBrightness(15);

    Serial.println();
    Serial.println("==============================================");
    Serial.println("Qbead dual-role entanglement window test");
    Serial.println("Upload the same sketch to both Qbeads.");
    Serial.println("Wait until both report: Notifications enabled");
    Serial.println("Then tap both Qbeads at the same time (within 100 ms).");
    Serial.println("==============================================");

    showIdle();
}

void loop()
{
    qbead.readIMU(false);

    // This processes, in order:
    // - a newly received remote Entangle notification,
    // - a newly detected local tap,
    // - request expiration after ENTANGLE_WINDOW_MS,
    // - a matched pair of requests.
    //
    // It returns true exactly once per successful match.
    if (qbead.entangle(ENTANGLED_STATE))
    {
        Serial.println("[SUCCESS] Entanglement handshake completed.");
        entangledDisplayActive = true;
        entangledAtMs = millis();
        showEntanglementSuccess();
    }

    if (!entangledDisplayActive)
    {
        if (qbead.localEntangleRequestPending)
        {
            showLocalRequestPending();
        }
        else if (qbead.remoteEntangleRequestPending)
        {
            showRemoteRequestPending();
        }
        else
        {
            showIdle();
        }
    }

    // Clear entanglement after the period, so that it can be re-demonstrated again.
    if (entangledDisplayActive &&
        (uint32_t)(millis() - entangledAtMs) >= ENTANGLED_DISPLAY_MS)
    {
        entangledDisplayActive = false;
        qbead.clearStates();
        showIdle();
        Serial.println("[INFO] Returned to idle.");
    }
}