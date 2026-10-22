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
    remoteSecondLocked = false;
    firstAxisAcked = false;
    secondAxisAcked = false;
    stage2AnnouncePending = false;

    ChshUtils::resetSimulation();

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

        // Guard applies to BOTH taps: the shake fires the tap interrupt,
        // which previously locked the first axis instantly.
        if (acceptedTap && stage2TapAllowed)
            ChshUtils::lockStage2Axis(bead);
    }
    else if (stage == ChshStage::Stage::Simulating)
    {
        // Start only when both beads have all four axes and both know
        // the other has them, so they start together.
        //   Alice shows the quantum result, Bob the classical (75%) one.
        if (!simStarted && ChshUtils::simulationReady())
        {
            // Same strategy on both beads: a = Alice's axes, b = Bob's.
            const Qbead::Axis &a0 = BEAD_ID == 0 ? localAxis : remoteAxis;
            const Qbead::Axis &a1 = BEAD_ID == 0 ? secondLocalAxis : remoteSecondAxis;
            const Qbead::Axis &b0 = BEAD_ID == 0 ? remoteAxis : localAxis;
            const Qbead::Axis &b1 = BEAD_ID == 0 ? remoteSecondAxis : secondLocalAxis;

            strategy = {
                {BlochVector(a0.x, a0.y, a0.z),
                 BlochVector(a1.x, a1.y, a1.z)},
                {BlochVector(b0.x, b0.y, b0.z),
                 BlochVector(b1.x, b1.y, b1.z)}};

            ChshUtils::computeProbabilityFromStrategy(PHI_PLUS, strategy);

            Serial.print("[SIM] Expected quantum win rate: ");
            Serial.print(100.f * qLimit, 2);
            Serial.println("%");

            rounds = 0;
            qWins = 0;
            cWins = 0;
            simDone = false;

            runStart = millis();
            fadeStartTime = runStart;
            simStarted = true;
        }

        // Nothing below may run before the start: with runStart == 0 the
        // schedule below thinks RUN_MS has long passed and plays all
        // N_TOTAL rounds in one loop() pass (instant result, using qP
        // from the previous run or zeros).
        if (simStarted)
        {
            if (rounds < N_TOTAL)
            {
                const uint32_t elapsed = millis() - runStart;
                const uint32_t target =
                    (elapsed >= RUN_MS)
                        ? N_TOTAL
                        : (uint32_t)((uint64_t)N_TOTAL * elapsed / RUN_MS);

                while (rounds < target)
                { // catch up to the schedule
                    bool qWin, cWin;
                    ChshUtils::playRound(qWin, cWin);
                    qWins += qWin;
                    cWins += cWin;
                    rounds++;
                }
            }
            else if (!simDone)
            {
                ChshUtils::printSimSummary();
                Serial.println("[SIM] Done. Tap to return to idle.");
                simDone = true;
            }

            bead.clear();
            if (BEAD_ID == 1)
            {
                bead.prMode.showDualLiquidFill(cWins, rounds - cWins, N_TOTAL, ChshUtils::simSuccessColor(), ChshUtils::simFailColor());
            }
            else
            {
                bead.prMode.showDualLiquidFill(qWins, rounds - qWins, N_TOTAL, ChshUtils::simSuccessColor(), ChshUtils::simFailColor());
            }
            // This must be after initial draw
            // TODO: Because of the inversion issue being resolved in the setBandColour it is now
            // very jankily fixed on this side at this getLegPixelColor issue.
            // Otherwise it gets the undercolour of the hemisphere inverted band
            // bead.prMode.showThresholdBand(fadeStartTime, bead.getLegPixelColor(0, THRESHOLD_BAND));
            bead.prMode.showThresholdBand(fadeStartTime, bead.getLegPixelColor(0, bead.nsections - THRESHOLD_BAND));
            bead.show();
        }

        // Leave only when finished AND the peer has our axes,
        // otherwise the peer would wait forever.
        if (acceptedTap && simDone && firstAxisAcked && secondAxisAcked)
        {
            stage = ChshStage::Stage::Collapsed;
            stage2AnnouncePending = false;
            ChshUtils::resetSimulation();
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
        else if (
            stage == ChshStage::Stage::AutomatedChoosing ||
            (stage == ChshStage::Stage::Simulating && !simStarted))
        {
            // Live preview, or the locked axis while waiting for the peer.
            lastDisplayMs = now;
            ChshUtils::drawSecondStage(bead);
        }
    }
}