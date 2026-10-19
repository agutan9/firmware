#include <Qbead.h>
#include <internal/BlochVector.h>
#include <internal/QbeadUtils.h>
#include <math.h>

#include "CHSH_Utils.h"
#include "CHSH_Comms.h"
#include "CHSH_Stage.h"

using namespace Qbead;
using namespace ChshStage;

Qbead::Qbead bead;

// -----------------------------------------------------------------------------
// Round preparation
// -----------------------------------------------------------------------------

void startRound()
{
    localLocked = false;
    remoteLocked = false;
    targetAvailable = false;

    firstChosen = false;
    firstRole = 255;
    firstMessage = 0;

    resultPending = false;
    resultMessage = 0;

    secondLocalLocked = false;
    stage2AnnouncePending = false;

    localInput =
        USE_RANDOM_INPUTS ? (uint8_t)random(2) : 0;
    ChshUtils::startInputFlash(localInput);

    remoteInput = 0;

    stage = ChshStage::Stage::Choosing;

    lastGestureMs = millis();
    ChshStage::lastSendMs = millis();

    Serial.println("[ROUND] New guided round.");

    Serial.print(
        BEAD_ID == 0
            ? "[INPUT] Alice x = "
            : "[INPUT] Bob y = ");
    Serial.println(localInput);

    Serial.println(
        "[ROUND] Either bead may freely choose the first axis.");

    ChshUtils::chshChoosing(bead);
}

// -----------------------------------------------------------------------------
// Simulated joint measurement
// -----------------------------------------------------------------------------

void measureBothIfReady()
{
    if (
        BEAD_ID != 0 ||
        stage != ChshStage::Stage::Choosing ||
        !firstChosen ||
        !localLocked ||
        !remoteLocked)
    {
        return;
    }

    const uint8_t x = localInput;
    const uint8_t y = remoteInput;

    const float pSame =
        ChshUtils::sameOutcomeProbability(
            localAxis,
            remoteAxis);

    const float pWin =
        (x & y) == 0 ? pSame : 1.f - pSame;

    const uint8_t a = (uint8_t)random(2);

    const float threshold =
        random(1000000L) / 1000000.f;

    const uint8_t b =
        threshold < pSame ? a : (a ^ 1U);

    Serial.print("[MEASURE] P(same) = ");
    Serial.println(pSame, 6);

    Serial.print("[MEASURE] P(win) for actual axes = ");
    Serial.println(pWin, 6);

    resultMessage =
        (uint32_t)a | ((uint32_t)b << 1) | ((uint32_t)x << 2) | ((uint32_t)y << 3);
    resultPending = true;

    ChshComms::sendMessage(
        BLEManager::CommandType::ChshResult,
        resultMessage,
        bead);

    ChshStage::lastSendMs = millis();

    ChshUtils::recordResult(
        x,
        y,
        a,
        b,
        roundsPlayed,
        roundsWon);

    ChshUtils::setCollapsedState(
        a,
        bead,
        stage);
}

// -----------------------------------------------------------------------------
// Arduino setup
// -----------------------------------------------------------------------------

void setup()
{
    bead.begin();
    bead.setBrightness(25);
    bead.testPixels();

    bead.clearStates();
    ChshUtils::showIdle(bead);

    bead.getBluetoothAddres(localAddress);

    Serial.print("[BOOT] Local Bluetooth address: ");
    ChshComms::printAddress(localAddress);
    Serial.println();

    Serial.println("[ROLE] Waiting for the other Qbead.");

    // Prototype seed, not a hardware random-number source.
    uint32_t seed = (uint32_t)micros();

    for (uint8_t i = 0; i < 6; ++i)
        seed = seed * 33UL + localAddress[i];

    randomSeed(seed);
}

// -----------------------------------------------------------------------------
// Arduino loop
// -----------------------------------------------------------------------------

void loop()
{
    bead.readIMU(false);

    // Consume the gestures exactly once.
    const bool tapped = bead.wasTapped();
    const bool shaken = ChshUtils::detectShake(bead);

    // Process received settings before the current local tap.
    // This defines the coordinator's ordering for close-together taps.
    for (uint8_t i = 0; i < 8; ++i)
    {
        BLEManager::DataPacket packet =
            bead.takeLatestPacket();

        if (packet.type == BLEManager::CommandType::None)
            break;

        ChshComms::handlePacket(packet, bead);
    }

    ChshComms::serviceIdentityExchange(bead);

    if (!roleAssigned || !peerRoleReady)
        return;

    const uint32_t now = millis();

    const bool acceptedTap =
        tapped &&
        (uint32_t)(now - lastGestureMs) >= TAP_GUARD_MS;

    if (acceptedTap)
        lastGestureMs = now;

    if (stage == ChshStage::Stage::Idle)
    {
        if (bead.entangle(acceptedTap, ENTANGLED_STATE))
            startRound();
    }
    else if (stage == ChshStage::Stage::Choosing)
    {
        if (acceptedTap && !localLocked)
            ChshUtils::lockLocalAxis(bead);

        measureBothIfReady();
    }
    else if (stage == ChshStage::Stage::AutomatedChoosing)
    {
        // Taps produced by the shake that brought us here are ignored.
        const bool stage2TapAllowed =
            (uint32_t)(now - stage2EnteredMs) >= STAGE2_TAP_GUARD_MS;

        if (acceptedTap && stage2TapAllowed && !secondLocalLocked)
            ChshUtils::lockSecondAxis(bead);
    }
    else if (stage == ChshStage::Stage::Simulating)
    {
        // TODO: stage-2 simulation goes here.

        if (acceptedTap)
        {
            stage = ChshStage::Stage::Idle;
            stage2AnnouncePending = false;
            bead.clearStates();
            ChshUtils::showIdle(bead);
            Serial.println("[STAGE2] Back to idle; entangle again to restart.");
        }
    }
    else // Collapsed
    {
        // Alice may only leave once Bob has confirmed the result.
        const bool resultSettled =
            BEAD_ID == 1 || !resultPending;

        if (shaken && resultSettled)
        {
            ChshUtils::enterSecondStage(bead, true);
        }
        else if (
            resultSettled &&
            bead.entangle(acceptedTap, ENTANGLED_STATE))
        {
            // Keep the outcome visible until the next entangle handshake.
            startRound();
        }
    }

    ChshComms::serviceResends(bead);

    if ((uint32_t)(now - lastDisplayMs) >= DISPLAY_MS)
    {
        if (stage == ChshStage::Stage::Choosing)
        {
            lastDisplayMs = now;
            ChshUtils::chshChoosing(bead);
        }
        else if (stage == ChshStage::Stage::AutomatedChoosing)
        {
            lastDisplayMs = now;
            ChshUtils::drawSecondStage(bead);
        }
    }
}