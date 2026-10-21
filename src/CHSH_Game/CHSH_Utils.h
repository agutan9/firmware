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

        // Sanity check: the locked axis should be close to what the
        // preview showed. ~180 deg here means the sign fix is wrong
        // for your fork (toggle CHSH_TAP_SIGN_FIX).
        const float previewDot = Qbead::dotProduct(
            outAxis,
            ChshUtils::previewAxisFor(bead));

        Serial.print("[LOCK] Angle to live preview = ");
        Serial.print(acosf(Qbead::clampUnit(previewDot)) * RAD_TO_DEG, 1);
        Serial.println(" degrees");

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

    // Textbook CHSH target for this bead and input (0 or 1).
    inline Qbead::Axis stage2Target(uint8_t index)
    {
        return Qbead::axisFromDegrees(
            STAGE2_TARGET_THETA[BEAD_ID][index & 1U],
            STAGE2_TARGET_PHI[BEAD_ID][index & 1U]);
    }

    inline void drawSecondStage(Qbead::Qbead &bead)
    {
        bead.clear();

        // Stage-1 outcome stays on the sphere as a reference.
        bead.displayCurrentStatesStatic();

        // Live preview first, so the targets stay visible on top
        // (same order as the stage-1 screen).
        if (!localLocked || !secondLocalLocked)
        {
            ChshUtils::addAxisOn(
                bead,
                ChshUtils::previewAxisFor(bead),
                Qbead::color(70, 70, 70));
        }

        const uint32_t yellow = Qbead::color(210, 207, 2);
        const uint32_t green = Qbead::color(0, 160, 40);

        // Textbook targets: yellow until that input's axis is chosen,
        // then green.
        if (localLocked)
        {
            ChshUtils::addAxisOn(
                bead,
                {localAxis.x, localAxis.y, localAxis.z},
                green);
        }
        else
        {
            ChshUtils::addAxisOn(
                bead,
                stage2Target(0),
                yellow);
        }

        if (secondLocalLocked)
        {
            ChshUtils::addAxisOn(
                bead,
                {secondLocalAxis.x, secondLocalAxis.y, secondLocalAxis.z},
                green);
        }
        else
        {
            ChshUtils::addAxisOn(
                bead,
                stage2Target(1),
                yellow);
        }

        bead.show();
    }

    // -------------------------------------------------------------------------
    // Stage-2 messages
    // -------------------------------------------------------------------------

    inline uint32_t stage2Value(uint32_t code, uint8_t index = 0)
    {
        return code |
               ((uint32_t)BEAD_ID << STAGE2_ROLE_SHIFT) |
               ((uint32_t)(index & 1U) << STAGE2_INDEX_SHIFT);
    }

    // index 0 -> localAxis, index 1 -> secondLocalAxis
    inline void sendStage2Axis(Qbead::Qbead &bead, uint8_t index)
    {
        ChshComms::sendAxisMessage(
            BLEManager::CommandType::ChshSimulating,
            stage2Value(STAGE2_AXIS, index),
            index == 0 ? localAxis : secondLocalAxis,
            bead);

        ChshStage::lastSendMs = millis();
    }

    inline void resetSimulation()
    {
        simStarted = false;
        simDone = false;

        rounds = 0;
        qWins = 0;
        cWins = 0;

        runStart = 0;
        fadeStartTime = 0;
    }

    // announce = true on the bead that was shaken,
    // false on the bead that follows the peer's announcement.
    inline void enterSecondStage(Qbead::Qbead &bead, bool announce)
    {
        stage = ChshStage::Stage::AutomatedChoosing;

        bead.clearStates();

        // Stage 2: both axes are chosen again.
        localLocked = false;
        remoteLocked = false;
        localAxis = {0.f, 0.f, 1.f};

        secondLocalLocked = false;
        secondLocalAxis = {0.f, 0.f, 1.f};

        remoteSecondLocked = false;
        remoteSecondAxis = {0.f, 0.f, 1.f};

        firstAxisAcked = false;
        secondAxisAcked = false;

        resetSimulation();

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

        Serial.println("[STAGE2] Tap to lock axis for input 0, then for input 1.");

        drawSecondStage(bead);
    }

    // Stage-2 lock. Does NOT use the stage-1 lockLocalAxis(): that one
    // sends ChshAxisLocked (ignored by Alice outside Choosing), may call
    // chooseFirst() and draws the stage-1 screen.
    inline void lockStage2Axis(Qbead::Qbead &bead)
    {
        const uint8_t index = localLocked ? 1 : 0;

        bool ok = index == 0
                      ? lockingLocalAxis(bead, localAxis, localLocked)
                      : lockingLocalAxis(bead, secondLocalAxis, secondLocalLocked);

        if (!ok)
            return;

        Serial.print("[STAGE2] Axis for input ");
        Serial.print(index);
        Serial.println(" locked; sending to peer.");

        {
            const Qbead::Axis &chosen =
                index == 0 ? localAxis : secondLocalAxis;

            const float dot =
                Qbead::dotProduct(chosen, stage2Target(index));

            Serial.print("[STAGE2] Error to textbook axis = ");
            Serial.print(acosf(Qbead::clampUnit(dot)) * RAD_TO_DEG, 1);
            Serial.println(" degrees");
        }

        // Resent by serviceResends() until STAGE2_AXIS_ACK arrives.
        sendStage2Axis(bead, index);

        // Simulating = "both local axes locked, waiting for the peer,
        // then running the simulation".
        if (index == 1)
            stage = ChshStage::Stage::Simulating;

        drawSecondStage(bead);
    }

    // True once both beads know all four axes AND each knows the other
    // received its axes. Both beads then start within one round trip.
    inline bool simulationReady()
    {
        return localLocked && secondLocalLocked &&
               remoteLocked && remoteSecondLocked &&
               firstAxisAcked && secondAxisAcked;
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
    inline float randomUnit()
    {
        return random(1000000L) / 1000000.f;
    }

    inline uint32_t simSuccessColor()
    {
        return BEAD_ID == 0
                   ? Qbead::color(0, 200, 0)
                   : Qbead::color(100, 100, 0);
    }

    inline uint32_t simFailColor()
    {
        return BEAD_ID == 0
                   ? Qbead::color(200, 0, 0)
                   : Qbead::color(200, 0, 0);
    }

    inline void printSimSummary()
    {
        const float qRate = rounds == 0 ? 0.f : (float)qWins / (float)rounds;
        const float cRate = rounds == 0 ? 0.f : (float)cWins / (float)rounds;

        Serial.print("[SIM] Rounds: ");
        Serial.println(rounds);

        Serial.print("[SIM] Quantum:   ");
        Serial.print(qWins);
        Serial.print(" wins = ");
        Serial.print(100.f * qRate, 2);
        Serial.print("%  (expected ");
        Serial.print(100.f * qLimit, 2);
        Serial.println("%)");

        Serial.print("[SIM] Classical: ");
        Serial.print(cWins);
        Serial.print(" wins = ");
        Serial.print(100.f * cRate, 2);
        Serial.println("%  (expected 75.00%)");

        // Win probability p relates to the CHSH value by S = 8 (p - 1/2).
        Serial.print("[SIM] S_quantum = ");
        Serial.println(8.f * (qRate - 0.5f), 3);

        Serial.println("[SIM] Classical bound 75% (S = 2), Tsirelson 85.36% (S = 2.828).");
    }

    inline void playRound(bool &qWin, bool &cWin)
    { // same (x,y) for both strategies
        uint32_t r = Qbead::updateRNG();
        // Xorshift's lowest bits are its weakest. Take highest bits
        int x = (r >> 31) & 1, y = (r >> 30) & 1;
        // This gives a 22-bit uniform, with resolution of about 2.4×10−72.4×10−7
        qWin = ((r & 0x3FFFFFu) * (1.0f / 4194304.0f)) < qP[x][y]; // 22 low-ish bits
        cWin = !(x & y);                                           // always answer 0
    }

    static inline float corr_E(const CorrelationDiag &T, const Qbead::BlochVector &a, const Qbead::BlochVector &b)
    {
        return a.x * T.x_corr * b.x + a.y * T.y_corr * b.y + a.z * T.z_corr * b.z;
    }

    // TODO: Normalize the four vectors before building Strategy, or add a debug assert that ∣E∣≤1∣E∣≤1
    inline void computeProbabilityFromStrategy(const CorrelationDiag &T, const Strategy &s)
    { // axes must be normalized
        float sum = 0;
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
            {
                float w = (x & y) ? -1.f : 1.f;
                qE[x][y] = corr_E(T, s.a[x], s.b[y]);
                qP[x][y] = 0.5f * (1.f + w * qE[x][y]);
                sum += qP[x][y];
            }
        // Long-run rate 1/4 ∑ p_{xy} = 1/2 + S/8
        qLimit = 0.25f * sum; // = 1/2 + S/8
    }
}

#endif // CHSH_UTILS_H