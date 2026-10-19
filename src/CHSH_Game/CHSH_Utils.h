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

    // announce = true on the bead that was shaken,
    // false on the bead that follows the peer's announcement.
    inline void enterSecondStage(Qbead::Qbead &bead, bool announce)
    {
        stage = ChshStage::Stage::AutomatedChoosing;

        secondLocalLocked = false;
        secondLocalAxis = {0.f, 0.f, 1.f};

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
                STAGE2_ENTER,
                bead);

            ChshStage::lastSendMs = millis();
        }

        Serial.println(
            announce
                ? "[STAGE2] Shake detected; entering stage 2."
                : "[STAGE2] Peer entered stage 2; following.");

        drawSecondStage(bead);
    }

    inline void lockSecondAxis(Qbead::Qbead &bead)
    {
        if (!lockingLocalAxis(bead, secondLocalAxis, secondLocalLocked))
            return;

        Serial.println("[STAGE2] Second axis locked; simulating.");

        stage = ChshStage::Stage::Simulating;

        drawSecondStage(bead);
    }
}

#endif // CHSH_UTILS_H