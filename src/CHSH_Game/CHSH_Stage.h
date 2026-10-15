#ifndef CHSH_STAGE_H
#define CHSH_STAGE_H

#include <Arduino.h>
#include <Qbead.h>
#include <internal/QbeadUtils.h>
#include <math.h>

namespace ChshStage
{
    enum class Stage : uint8_t
    {
        Idle,
        Choosing,
        Collapsed
    };

    // -----------------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------------

    static constexpr uint32_t ENTANGLED_STATE = 1;

    static constexpr uint32_t IDENTITY_TX_MS = 150;
    static constexpr uint32_t RESEND_MS = 250;
    static constexpr uint32_t TAP_GUARD_MS = 350;
    static constexpr uint32_t DISPLAY_MS = 40;

    // true:
    // Generate local random inputs.
    // Show a 45-degree or 135-degree target according to x AND y.
    //
    // false:
    // Always demonstrate the 45-degree, equal-outcome case.
    static constexpr bool USE_RANDOM_INPUTS = true;

    // -----------------------------------------------------------------------------
    // Application state
    // -----------------------------------------------------------------------------

    ChshStage::Stage stage = ChshStage::Stage::Idle;

    // Automatically assigned:
    // 0 = Alice / simulation coordinator
    // 1 = Bob
    //
    // This ID does not determine which bead chooses first.
    uint8_t BEAD_ID = 255;
    bool roleAssigned = false;

    uint8_t localAddress[6] = {0};
    uint8_t remoteAddress[6] = {0};

    uint8_t remoteAddressParts = 0;
    bool peerRoleReady = false;

    uint8_t identityTxSlot = 0;
    uint32_t lastIdentityTxMs = 0;

    // Axis is defined in QbeadUtils.h.
    Qbead::Axis localAxis = {0.f, 0.f, 1.f};
    Qbead::Axis remoteAxis = {0.f, 0.f, 1.f};
    Qbead::Axis targetAxis = {0.f, 0.f, 1.f};

    bool localLocked = false;
    bool remoteLocked = false;
    bool targetAvailable = false;

    // Input x on Alice, input y on Bob.
    uint8_t localInput = 0;
    uint8_t remoteInput = 0;

    // Authoritative ordering announced by the coordinator.
    bool firstChosen = false;
    uint8_t firstRole = 255;
    uint32_t firstMessage = 0;

    uint32_t lastGestureMs = 0;
    uint32_t lastSendMs = 0;
    uint32_t lastDisplayMs = 0;

    bool resultPending = false;
    uint32_t resultMessage = 0;

    uint32_t roundsPlayed = 0;
    uint32_t roundsWon = 0;
}

#endif // CHSH_STAGE_H