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
        // Input 0 -> cyan
        // Input 1 -> orange
        flash.pixelColor =
            input == 0
                ? Qbead::color(0, 100, 100)
                : Qbead::color(150, 50, 0);
    }

    // Returns true while the flash owns the display.
    inline bool showInputFlash(Qbead::Qbead &bead)
    {
        InputFlash &flash = inputFlashState();

        if (!flash.active)
            return false;

        constexpr uint32_t flashDurationMs = 300;

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
        for (uint8_t i = 0; i < bead.innerStateCount; ++i)
        {
            bead.setBloch_deg(
                bead.innerStates[i],
                bead.stateColours[i]);
        }

        if (localLocked)
        {
            // Locked measurement axis.
            ChshUtils::drawAxisOn(
                bead,
                Qbead::oppositeAxis(localAxis),
                Qbead::color(0, 0, 15));

            ChshUtils::drawAxisOn(
                bead,
                localAxis,
                Qbead::color(0, 0, 150));
        }
        else
        {
            const Qbead::Axis live =
                ChshUtils::previewAxisFor(bead);

            // White: current selectable measurement axis.
            ChshUtils::drawAxisOn(
                bead,
                Qbead::oppositeAxis(live),
                Qbead::color(10, 10, 10));

            ChshUtils::drawAxisOn(
                bead,
                live,
                Qbead::color(90, 90, 90));

            // Green: suggested axis for the second bead.
            if (targetAvailable)
            {
                ChshUtils::drawAxisOn(
                    bead,
                    Qbead::oppositeAxis(targetAxis),
                    Qbead::color(0, 15, 0));

                ChshUtils::drawAxisOn(
                    bead,
                    targetAxis,
                    Qbead::color(0, 150, 0));
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

    void updateSecondTarget(Qbead::Qbead &bead)
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

        lastSendMs = millis();
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

        lastSendMs = millis();
    }

    void lockLocalAxis(Qbead::Qbead &bead)
    {
        Qbead::Axis tappedAxis = {
            bead.x_whentapped,
            bead.y_whentapped,
            bead.z_whentapped};

        if (!Qbead::normalise(tappedAxis))
        {
            Serial.println("[LOCK] Invalid tap axis; try again.");
            return;
        }

        localAxis = ChshUtils::quantiseAxis(tappedAxis);
        localLocked = true;

        float thetaDeg;
        float phiDeg;

        Qbead::axisAngles(
            localAxis,
            thetaDeg,
            phiDeg);

        Serial.print("[LOCK] Actual theta = ");
        Serial.print(thetaDeg);
        Serial.print(", phi = ");
        Serial.println(phiDeg);

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

        ChshUtils::chshChoosing(
            bead);
    }
}

#endif // CHSH_UTILS_H