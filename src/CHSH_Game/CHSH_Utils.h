#ifndef CHSH_UTILS_H
#define CHSH_UTILS_H

#include <Arduino.h>
#include <Qbead.h>
#include <internal/BlochVector.h>
#include <internal/QbeadUtils.h>
#include <math.h>

#include "CHSH_Stage.h"

using namespace ChshStage;

namespace ChshComms
{
    inline void sendMessage(
        BLEManager::CommandType type,
        uint32_t value,
        Qbead::Qbead &bead,
        uint32_t theta = 0,
        uint32_t phi = 0);

    inline void sendAxisMessage(
        BLEManager::CommandType type,
        uint32_t value,
        const Qbead::Axis &axis,
        Qbead::Qbead &bead);
}

namespace ChshUtils
{

    struct InputFlash
    {
        bool active = false;
        uint32_t startedAtMs = 0;
        uint32_t pixelColor = 0;
    };

    inline InputFlash &inputFlashState()
    {
        static InputFlash flash;
        return flash;
    }

    inline void startInputFlash(uint8_t input)
    {
        InputFlash &flash = inputFlashState();

        flash.active = true;
        flash.startedAtMs = millis();

        // Choose whichever colors you prefer.
        // Input 0 -> blue
        // Input 1 -> red
        flash.pixelColor =
            input == 0
                ? Qbead::color(0, 0, 100)
                : Qbead::color(100, 0, 0);
    }

    // Returns true while the flash owns the display.
    inline bool showInputFlash(Qbead::Qbead &bead)
    {
        InputFlash &flash = inputFlashState();

        if (!flash.active)
            return false;

        constexpr uint32_t flashDurationMs = 500;

        if (
            (uint32_t)(millis() - flash.startedAtMs) >= flashDurationMs)
        {
            flash.active = false;
            return false;
        }

        bead.clear();

        for (uint16_t i = 0; i < bead.pixels.numPixels(); ++i)
        {
            bead.pixels.setPixelColor(
                i,
                flash.pixelColor);
        }

        bead.show();

        return true;
    }

    // Add an axis marker to the existing LED frame.
    // Uses the same pixel selection as Qbead::setBloch_deg().
    inline void addAxisOn(
        Qbead::Qbead &bead,
        const Qbead::Axis &axis,
        uint32_t pixelColor)
    {
        float thetaDeg;
        float phiDeg;

        Qbead::axisAngles(
            axis,
            thetaDeg,
            phiDeg);

        const float thetaSection =
            thetaDeg / bead.theta_quant;

        int leg;
        int section;

        if (thetaSection < 0.5f)
        {
            leg = 0;
            section = 0;
        }
        else if (thetaSection > bead.nsections - 0.5f)
        {
            leg = 0;
            section = bead.nsections;
        }
        else
        {
            section = (int)roundf(thetaSection);

            if (section > bead.nsections - 1)
                section = bead.nsections - 1;

            leg = (int)roundf(
                phiDeg / bead.phi_quant);

            leg %= bead.nlegs;
        }

        // Add to the current pixel instead of replacing its color.
        bead.addLegPixelColor(
            leg,
            section,
            pixelColor);
    }
    // -------------------------------------------------------------------------
    // Live measurement axis
    // -------------------------------------------------------------------------

    inline Qbead::Axis previewAxisFor(Qbead::Qbead &bead)
    {
        Qbead::Axis axis;

        if (bead.gravity.isInitialised)
        {
            axis = {
                bead.gravity.gravityEstimate[0],
                bead.gravity.gravityEstimate[1],
                bead.gravity.gravityEstimate[2]};
        }
        else
        {
            axis = {
                bead.x,
                bead.y,
                bead.z};
        }

        if (!Qbead::normalise(axis))
            axis = {0.f, 0.f, 1.f};

        return axis;
    }

    // -------------------------------------------------------------------------
    // Display helpers
    // -------------------------------------------------------------------------

    inline void drawAxisOn(
        Qbead::Qbead &bead,
        const Qbead::Axis &axis,
        uint32_t pixelColor)
    {
        float thetaDeg;
        float phiDeg;

        Qbead::axisAngles(
            axis,
            thetaDeg,
            phiDeg);

        bead.setBloch_deg(
            thetaDeg,
            phiDeg,
            pixelColor);
    }

    inline void chshChoosing(Qbead::Qbead &bead)
    {
        if (showInputFlash(bead))
            return;
        bead.clear();

        // Static visualization of the prepared entangled state.
        bead.displayCurrentStatesStatic();

        if (localLocked)
        {
            ChshUtils::drawAxisOn(
                bead,
                localAxis,
                Qbead::color(210, 207, 2));
        }
        else
        {
            const Qbead::Axis live =
                ChshUtils::previewAxisFor(bead);

            ChshUtils::addAxisOn(
                bead,
                live,
                Qbead::color(70, 70, 70));

            if (targetAvailable)
            {
                ChshUtils::addAxisOn(
                    bead,
                    targetAxis,
                    Qbead::color(0, 120, 0));
            }
        }

        // Send the complete frame once.
        bead.show();
    }

    inline void showIdle(Qbead::Qbead &bead)
    {
        bead.clear();
        bead.setBloch_deg(
            0,
            0,
            Qbead::color(15, 0, 15));
        bead.show();
    }

    inline void setCollapsedState(
        uint8_t outcome,
        Qbead::Qbead &bead,
        ChshStage::Stage &stage)
    {
        bead.clearStates();

        // Result display in measurement-basis coordinates:
        // 0 -> north pole
        // 1 -> south pole
        Qbead::BlochVector state(
            outcome == 0 ? 0.f : 180.f,
            0.f);

        bead.addState(state);

        // Outcome 0 -> blue; outcome 1 -> red.
        bead.displayCurrentStatesStatic(outcome);

        stage = ChshStage::Stage::Collapsed;

        Serial.print("[COLLAPSE] Local outcome: ");
        Serial.println(outcome);

        Serial.println(
            outcome == 0
                ? "[COLLAPSE] North pole / up."
                : "[COLLAPSE] South pole / down.");
    }

    // -------------------------------------------------------------------------
    // Axis encoding
    // -------------------------------------------------------------------------
    //
    // Expects a finite, normalized axis.
    //
    // bits 0..8:   theta in whole degrees
    // bits 9..17:  phi in whole degrees
    //
    // Higher bits are ignored by decodeAxis().

    inline uint32_t encodeAxis(const Qbead::Axis &axis)
    {
        float thetaDeg;
        float phiDeg;

        Qbead::axisAngles(
            axis,
            thetaDeg,
            phiDeg);

        const uint32_t thetaInteger =
            (uint32_t)lroundf(thetaDeg);

        const uint32_t phiInteger =
            (uint32_t)lroundf(phiDeg) % 360UL;

        return thetaInteger | (phiInteger << 9);
    }

    inline bool decodeAxis(
        uint32_t value,
        Qbead::Axis &axis)
    {
        const uint32_t thetaInteger =
            value & 0x1FFUL;

        const uint32_t phiInteger =
            (value >> 9) & 0x1FFUL;

        if (thetaInteger > 180 || phiInteger >= 360)
            return false;

        axis = Qbead::axisFromDegrees(
            (float)thetaInteger,
            (float)phiInteger);

        return true;
    }

    inline Qbead::Axis quantiseAxis(Qbead::Axis axis)
    {
        Qbead::Axis decoded = axis;

        ChshUtils::decodeAxis(
            ChshUtils::encodeAxis(axis),
            decoded);

        return decoded;
    }

    // -------------------------------------------------------------------------
    // Guided target
    // -------------------------------------------------------------------------

    inline Qbead::Axis guidedTarget(
        const Qbead::Axis &firstAxis,
        bool requireDifferent)
    {
        constexpr float invSqrtTwo = 0.70710678118f;

        // |Phi+> correlation-matching direction.
        const Qbead::Axis matching = {
            firstAxis.x,
            -firstAxis.y,
            firstAxis.z};

        const Qbead::Axis perpendicular =
            Qbead::perpendicularTo(matching);

        // Same required:      45 degrees from matching.
        // Different required: 135 degrees from matching.
        const float parallelSign =
            requireDifferent ? -1.f : 1.f;

        Qbead::Axis target = {
            invSqrtTwo *
                (parallelSign * matching.x + perpendicular.x),

            invSqrtTwo *
                (parallelSign * matching.y + perpendicular.y),

            invSqrtTwo *
                (parallelSign * matching.z + perpendicular.z)};

        Qbead::normalise(target);

        return target;
    }

    // -------------------------------------------------------------------------
    // Measurement probability
    // -------------------------------------------------------------------------

    inline float sameOutcomeProbability(
        const Qbead::Axis &localAxis,
        const Qbead::Axis &remoteAxis)
    {
        // |Phi+> correlation tensor: diag(+1, -1, +1).
        const float correlation =
            localAxis.x * remoteAxis.x - localAxis.y * remoteAxis.y + localAxis.z * remoteAxis.z;

        return 0.5f * (1.f + Qbead::clampUnit(correlation));
    }

    // -------------------------------------------------------------------------
    // Score reporting
    // -------------------------------------------------------------------------

    inline void recordResult(
        uint8_t x,
        uint8_t y,
        uint8_t a,
        uint8_t b,
        uint32_t &roundsPlayed,
        uint32_t &roundsWon)
    {
        const bool won = ((a ^ b) == (x & y));

        ++roundsPlayed;

        if (won)
            ++roundsWon;

        Serial.print("[RESULT] x = ");
        Serial.print(x);
        Serial.print(", y = ");
        Serial.print(y);
        Serial.print(", a = ");
        Serial.print(a);
        Serial.print(", b = ");
        Serial.println(b);

        Serial.print("[RESULT] Required: ");
        Serial.println(
            (x & y) == 0
                ? "equal outcomes"
                : "different outcomes");

        Serial.println(
            won ? "[RESULT] WIN" : "[RESULT] LOSS");

        Serial.print("[SCORE] ");
        Serial.print(roundsWon);
        Serial.print("/");
        Serial.print(roundsPlayed);
        Serial.print(" = ");

        Serial.print(
            100.f * (float)roundsWon / (float)roundsPlayed,
            2);

        Serial.println("%");
    }

    // -----------------------------------------------------------------------------
    // First-setting announcement and second target
    // -----------------------------------------------------------------------------

    inline void updateSecondTarget(Qbead::Qbead &bead)
    {
        targetAvailable = false;

        // Only the bead that goes second receives a target.
        if (!firstChosen || firstRole == BEAD_ID)
            return;

        const bool requireDifferent =
            (localInput & remoteInput) != 0;

        targetAxis = ChshUtils::guidedTarget(
            remoteAxis,
            requireDifferent);

        targetAvailable = true;

        float thetaDeg;
        float phiDeg;

        Qbead::axisAngles(
            targetAxis,
            thetaDeg,
            phiDeg);

        Serial.println("[TARGET] First setting received.");

        Serial.println(
            requireDifferent
                ? "[TARGET] 135-degree target; different outcomes required."
                : "[TARGET] 45-degree target; equal outcomes required.");

        Serial.print("[TARGET] theta = ");
        Serial.print(thetaDeg);
        Serial.print(", phi = ");
        Serial.println(phiDeg);

        if (!localLocked)
        {
            Serial.println(
                "[TARGET] Align white with green, then tap.");
        }
        else
        {
            Serial.println(
                "[TARGET] Local axis was already locked; keeping it.");
        }

        chshChoosing(bead);
    }

    inline void chooseFirst(
        uint8_t role,
        Qbead::Qbead &bead)
    {
        if (BEAD_ID != 0 || firstChosen)
            return;

        firstChosen = true;
        firstRole = role;

        const Qbead::Axis &axis =
            role == BEAD_ID ? localAxis : remoteAxis;

        const uint8_t input =
            role == BEAD_ID ? localInput : remoteInput;

        // Bit 0: first input
        // Bit 1: first bead's role
        firstMessage =
            (uint32_t)input | ((uint32_t)role << 1);

        Serial.print("[FIRST] First accepted bead ID: ");
        Serial.println(firstRole);

        ChshUtils::updateSecondTarget(bead);

        ChshComms::sendAxisMessage(
            BLEManager::CommandType::ChshFirstSetting,
            firstMessage,
            axis,
            bead);

        ChshStage::lastSendMs = millis();
    }

    // -----------------------------------------------------------------------------
    // Locking the actual measurement axis
    // -----------------------------------------------------------------------------

    inline void sendLocalLock(Qbead::Qbead &bead)
    {
        // Bit 0: local input
        // Bit 1: sender's role
        const uint32_t flags =
            (uint32_t)localInput | ((uint32_t)BEAD_ID << 1);

        ChshComms::sendAxisMessage(
            BLEManager::CommandType::ChshAxisLocked,
            flags,
            localAxis,
            bead);

        ChshStage::lastSendMs = millis();
    }

    // Returns false (and leaves the outputs untouched) if the tap axis is
    // unusable. Callers MUST stop when this returns false.
    inline bool lockingLocalAxis(
        Qbead::Qbead &bead,
        Qbead::Axis &outAxis,
        bool &outLocked)
    {
        Qbead::Axis tappedAxis = {
            bead.x_whentapped,
            bead.y_whentapped,
            bead.z_whentapped};

        if (!Qbead::normalise(tappedAxis))
        {
            Serial.println("[LOCK] Invalid tap axis; try again.");
            return false;
        }

        outAxis = ChshUtils::quantiseAxis(tappedAxis);
        outLocked = true;

        float thetaDeg;
        float phiDeg;

        Qbead::axisAngles(
            outAxis,
            thetaDeg,
            phiDeg);

        Serial.print("[LOCK] Actual theta = ");
        Serial.print(thetaDeg);
        Serial.print(", phi = ");
        Serial.println(phiDeg);

        return true;
    }

    // Stage 1 only. Same behaviour as before stage 2 was introduced.
    inline void lockLocalAxis(Qbead::Qbead &bead)
    {
        if (!lockingLocalAxis(bead, localAxis, localLocked))
            return;

        if (targetAvailable)
        {
            const float dot =
                Qbead::dotProduct(localAxis, targetAxis);

            const float errorDegrees =
                acosf(Qbead::clampUnit(dot)) * RAD_TO_DEG;

            Serial.print("[LOCK] Target error = ");
            Serial.print(errorDegrees, 2);
            Serial.println(" degrees");
        }

        if (BEAD_ID == 0)
        {
            if (!firstChosen)
                ChshUtils::chooseFirst(0, bead);

            // Alice's first setting is sent through TAG_FIRST.
            // If Bob went first, only Alice needs both axes
            // to generate the simulated result.
        }
        else
        {
            sendLocalLock(bead);
        }

        ChshUtils::chshChoosing(bead);
    }

    // -------------------------------------------------------------------------
    // Shake detection
    // -------------------------------------------------------------------------
    //
    // Call once per loop, after bead.readIMU(). Uses the raw (unfiltered)
    // acceleration bead.rx/ry/rz in units of g. Returns true once per shake.

    inline bool detectShake(Qbead::Qbead &bead)
    {
        static uint8_t peaks = 0;
        static uint32_t firstPeakMs = 0;
        static uint32_t lastPeakMs = 0;

        const uint32_t now = millis();

        if (peaks > 0 && (uint32_t)(now - firstPeakMs) > SHAKE_WINDOW_MS)
            peaks = 0;

        const float mag = sqrtf(
            bead.rx * bead.rx +
            bead.ry * bead.ry +
            bead.rz * bead.rz);

        if (fabsf(mag - 1.f) < SHAKE_DEVIATION_G)
            return false;

        if (peaks > 0 && (uint32_t)(now - lastPeakMs) < SHAKE_PEAK_GAP_MS)
            return false;

        if (peaks == 0)
            firstPeakMs = now;

        lastPeakMs = now;

        if (++peaks < SHAKE_PEAKS)
            return false;

        peaks = 0;
        return true;
    }

    // -------------------------------------------------------------------------
    // Stage 2
    // -------------------------------------------------------------------------

    inline void drawSecondStage(Qbead::Qbead &bead)
    {
        bead.clear();

        // Stage-1 outcome stays on the sphere as a reference.
        bead.displayCurrentStatesStatic();

        if (secondLocalLocked)
        {
            ChshUtils::addAxisOn(
                bead,
                secondLocalAxis,
                Qbead::color(210, 207, 2));
        }
        else
        {
            ChshUtils::addAxisOn(
                bead,
                ChshUtils::previewAxisFor(bead),
                Qbead::color(70, 70, 70));
        }

        bead.show();
    }

    // -------------------------------------------------------------------------
    // Stage-2 messages
    // -------------------------------------------------------------------------

    inline uint32_t stage2Value(uint32_t code)
    {
        return code | ((uint32_t)BEAD_ID << STAGE2_ROLE_SHIFT);
    }

    inline void sendSecondAxis(Qbead::Qbead &bead)
    {
        ChshComms::sendAxisMessage(
            BLEManager::CommandType::ChshSimulating,
            stage2Value(STAGE2_AXIS),
            secondLocalAxis,
            bead);

        ChshStage::lastSendMs = millis();
    }

    // announce = true on the bead that was shaken,
    // false on the bead that follows the peer's announcement.
    inline void enterSecondStage(Qbead::Qbead &bead, bool announce)
    {
        stage = ChshStage::Stage::AutomatedChoosing;

        secondLocalLocked = false;
        secondLocalAxis = {0.f, 0.f, 1.f};

        remoteSecondLocked = false;
        remoteSecondAxis = {0.f, 0.f, 1.f};
        secondAxisAcked = false;

        simStarted = false;
        simDone = false;
        simRounds = 0;
        simWins = 0;
        simExpected = 0.f;

        stage2EnteredMs = millis();
        stage2AnnouncePending = announce;

        // Both beads are past the result screen, so the result handshake
        // is complete (Bob only leaves Collapsed after acknowledging it).
        resultPending = false;

        // Drop any tap generated by the shake itself.
        bead.wasTapped();

        if (announce)
        {
            ChshComms::sendMessage(
                BLEManager::CommandType::ChshSimulating,
                stage2Value(STAGE2_ENTER),
                bead);

            ChshStage::lastSendMs = millis();
        }

        Serial.println(
            announce
                ? "[STAGE2] Shake detected; entering stage 2."
                : "[STAGE2] Peer entered stage 2; following.");

        Serial.println("[STAGE2] Choose your second axis, then tap.");

        drawSecondStage(bead);
    }

    inline void lockSecondAxis(Qbead::Qbead &bead)
    {
        if (!lockingLocalAxis(bead, secondLocalAxis, secondLocalLocked))
            return;

        Serial.println("[STAGE2] Second axis locked; sending to peer.");

        // Resent by serviceResends() until STAGE2_AXIS_ACK arrives.
        sendSecondAxis(bead);

        // Simulating = "locked, waiting for / running the simulation".
        stage = ChshStage::Stage::Simulating;

        drawSecondStage(bead);
    }

    // -------------------------------------------------------------------------
    // Stage-2 simulation
    // -------------------------------------------------------------------------
    //
    // Measurement settings after both stages:
    //
    //   Alice: A[localInput]  = localAxis        (stage 1)
    //          A[1-localInput]= secondLocalAxis  (stage 2)
    //   Bob:   B[remoteInput] = remoteAxis       (stage 1)
    //          B[1-remoteInput]= remoteSecondAxis (stage 2)
    //
    // Only Alice needs all four. Bob's classical strategy needs none.

    inline void aliceSettings(Qbead::Axis A[2], Qbead::Axis B[2])
    {
        A[localInput & 1U] = localAxis;
        A[(localInput & 1U) ^ 1U] = secondLocalAxis;

        B[remoteInput & 1U] = remoteAxis;
        B[(remoteInput & 1U) ^ 1U] = remoteSecondAxis;
    }

    inline float randomUnit()
    {
        return random(1000000L) / 1000000.f;
    }

    // Exact win probability of the quantum strategy for the four axes,
    // averaged over uniformly random inputs x, y.
    inline float quantumExpectedWin(const Qbead::Axis A[2], const Qbead::Axis B[2])
    {
        float sum = 0.f;

        for (uint8_t x = 0; x < 2; ++x)
        {
            for (uint8_t y = 0; y < 2; ++y)
            {
                const float pSame =
                    sameOutcomeProbability(A[x], B[y]);

                sum += (x & y) == 0 ? pSame : 1.f - pSame;
            }
        }

        return 0.25f * sum;
    }

    // One round of the quantum game on |Phi+>. Returns true on a win.
    inline bool quantumRound(const Qbead::Axis A[2], const Qbead::Axis B[2])
    {
        const uint8_t x = (uint8_t)random(2);
        const uint8_t y = (uint8_t)random(2);

        const float pSame =
            sameOutcomeProbability(A[x], B[y]);

        const uint8_t a = (uint8_t)random(2);
        const uint8_t b = randomUnit() < pSame ? a : (a ^ 1U);

        return (a ^ b) == (x & y);
    }

    // One round of the optimal deterministic classical strategy:
    // both players always answer 0. Wins unless x = y = 1, i.e. 75%.
    inline bool classicalRound()
    {
        const uint8_t x = (uint8_t)random(2);
        const uint8_t y = (uint8_t)random(2);

        const uint8_t a = 0;
        const uint8_t b = 0;

        return (a ^ b) == (x & y);
    }

    // Win-rate bar: lights the fraction of pixels equal to the win rate.
    inline void drawWinRate(Qbead::Qbead &bead, float rate, uint32_t pixelColor)
    {
        bead.clear();

        const uint16_t n = bead.pixels.numPixels();
        uint16_t lit = (uint16_t)lroundf(Qbead::clampUnit(rate) * n);

        if (lit > n)
            lit = n;

        for (uint16_t i = 0; i < lit; ++i)
            bead.pixels.setPixelColor(i, pixelColor);

        bead.show();
    }

    inline uint32_t simColor()
    {
        // Alice / quantum: cyan. Bob / classical: orange.
        return BEAD_ID == 0
                   ? Qbead::color(0, 90, 120)
                   : Qbead::color(140, 60, 0);
    }

    inline void printSimSummary()
    {
        const float rate =
            simRounds == 0 ? 0.f : (float)simWins / (float)simRounds;

        Serial.print(
            BEAD_ID == 0
                ? "[SIM] Quantum (Alice): "
                : "[SIM] Classical (Bob): ");
        Serial.print(simWins);
        Serial.print("/");
        Serial.print(simRounds);
        Serial.print(" = ");
        Serial.print(100.f * rate, 2);
        Serial.println("%");

        Serial.print("[SIM] Expected: ");
        Serial.print(100.f * simExpected, 2);
        Serial.println("%");

        // Win probability p relates to the CHSH value by S = 8 (p - 1/2).
        Serial.print("[SIM] S = 8(p - 1/2) = ");
        Serial.println(8.f * (rate - 0.5f), 3);

        Serial.println("[SIM] Classical bound 75% (S = 2), Tsirelson 85.36% (S = 2.828).");
    }

    inline void startSimulation(Qbead::Qbead &bead)
    {
        simStarted = true;
        simDone = false;
        simRounds = 0;
        simWins = 0;

        if (BEAD_ID == 0)
        {
            Qbead::Axis A[2];
            Qbead::Axis B[2];
            aliceSettings(A, B);

            simExpected = quantumExpectedWin(A, B);

            Serial.println("[SIM] Both second axes known; running quantum CHSH.");

            for (uint8_t i = 0; i < 2; ++i)
            {
                float t, ph;

                Qbead::axisAngles(A[i], t, ph);
                Serial.print("[SIM] A");
                Serial.print(i);
                Serial.print(": theta = ");
                Serial.print(t);
                Serial.print(", phi = ");
                Serial.println(ph);

                Qbead::axisAngles(B[i], t, ph);
                Serial.print("[SIM] B");
                Serial.print(i);
                Serial.print(": theta = ");
                Serial.print(t);
                Serial.print(", phi = ");
                Serial.println(ph);
            }
        }
        else
        {
            simExpected = 0.75f;

            Serial.println("[SIM] Both second axes known; running classical CHSH (a = b = 0).");
        }

        drawWinRate(bead, 0.f, simColor());
    }

    // Runs SIM_BATCH rounds per call so loop() stays responsive
    // (BLE packets, resends and the display keep being serviced).
    inline void stepSimulation(Qbead::Qbead &bead)
    {
        if (!simStarted || simDone)
            return;

        Qbead::Axis A[2];
        Qbead::Axis B[2];

        if (BEAD_ID == 0)
            aliceSettings(A, B);

        for (uint16_t i = 0; i < SIM_BATCH && simRounds < SIM_ROUNDS; ++i)
        {
            const bool won =
                BEAD_ID == 0 ? quantumRound(A, B) : classicalRound();

            ++simRounds;

            if (won)
                ++simWins;
        }

        drawWinRate(
            bead,
            (float)simWins / (float)simRounds,
            simColor());

        if (simRounds >= SIM_ROUNDS)
        {
            simDone = true;
            printSimSummary();
            Serial.println("[SIM] Done. Tap to return to idle.");
        }
    }
}

#endif // CHSH_UTILS_H