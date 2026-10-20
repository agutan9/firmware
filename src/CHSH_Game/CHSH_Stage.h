#ifndef CHSH_STAGE_H
#define CHSH_STAGE_H

#include <Arduino.h>
#include <Qbead.h>
#include <internal/QbeadUtils.h>
#include <internal/BlochVector.h>
#include <math.h>

namespace ChshStage
{
    enum class Stage : uint8_t
    {
        Idle,
        Choosing,
        Collapsed,
        AutomatedChoosing,
        Simulating
    };

    // -----------------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------------

    static constexpr uint32_t ENTANGLED_STATE = 1;

    static constexpr uint32_t IDENTITY_TX_MS = 150;
    static constexpr uint32_t RESEND_MS = 250;
    static constexpr uint32_t TAP_GUARD_MS = 350;
    static constexpr uint32_t DISPLAY_MS = 40;

    // Shake detection (raw accelerometer, units of g).
    // A "peak" is a sample whose |a| deviates from 1 g by more than
    // SHAKE_DEVIATION_G. SHAKE_PEAKS peaks within SHAKE_WINDOW_MS = shake.
    static constexpr float SHAKE_DEVIATION_G = 1.0f;
    static constexpr uint8_t SHAKE_PEAKS = 4;
    static constexpr uint32_t SHAKE_WINDOW_MS = 1000;
    static constexpr uint32_t SHAKE_PEAK_GAP_MS = 60;

    // Shaking fires the IMU tap interrupt as well. Ignore taps for this
    // long after entering stage 2, otherwise the shake itself locks the
    // second axis immediately.
    static constexpr uint32_t STAGE2_TAP_GUARD_MS = 1200;

    // Values carried by CommandType::ChshSimulating.
    //
    // bits 0..2: message code (below)
    // bit  3:    sender's role (BEAD_ID)
    //
    // STAGE2_AXIS carries the sender's second axis in theta/phi.
    static constexpr uint32_t STAGE2_ENTER = 1;     // "go to stage 2"
    static constexpr uint32_t STAGE2_ENTER_ACK = 2; // "I am in stage 2"
    static constexpr uint32_t STAGE2_AXIS = 3;      // "my second axis"
    static constexpr uint32_t STAGE2_AXIS_ACK = 4;  // "got your second axis"

    static constexpr uint32_t STAGE2_CODE_MASK = 0x7UL;
    static constexpr uint8_t STAGE2_ROLE_SHIFT = 3;

    // Stage-2 simulation.
    // Alice: quantum CHSH with her two axes and Bob's two axes.
    // Bob:   classical optimal strategy (a = b = 0), wins 75%.
    static constexpr uint32_t SIM_ROUNDS = 2000;
    static constexpr uint16_t SIM_BATCH = 25; // rounds per loop() pass

    // true:
    // Generate local random inputs.
    // Show a 45-degree or 135-degree target according to x AND y.
    //
    // false:
    // Always demonstrate the 45-degree, equal-outcome case.
    static constexpr bool USE_RANDOM_INPUTS = true;
    const uint8_t THRESHOLD_BAND = (uint8_t)(0.75f * QB_NBANDS);
    constexpr uint32_t N_TOTAL = 5000;
    constexpr uint32_t RUN_MS = 50000; // total animation time

    uint32_t runStart = 0;
    uint32_t rounds = 0, qWins = 0, cWins = 0;
    uint32_t fadeStartTime = 0;

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
    Qbead::Axis secondLocalAxis = {0.f, 0.f, 1.f};
    Qbead::Axis remoteAxis = {0.f, 0.f, 1.f};
    Qbead::Axis targetAxis = {0.f, 0.f, 1.f};

    bool localLocked = false;
    bool secondLocalLocked = false;
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

    // Stage 2 handshake: the bead that was shaken keeps announcing
    // STAGE2_ENTER until the peer answers with STAGE2_ENTER_ACK.
    bool stage2AnnouncePending = false;
    uint32_t stage2EnteredMs = 0;

    // Peer's second axis (received over BLE) and whether the peer has
    // confirmed receiving ours.
    Qbead::Axis remoteSecondAxis = {0.f, 0.f, 1.f};
    bool remoteSecondLocked = false;
    bool secondAxisAcked = false;

    // Simulation progress.
    bool simStarted = false;
    bool simDone = false;
    uint32_t simRounds = 0;
    uint32_t simWins = 0;
    float simExpected = 0.f; // exact expected win probability

    uint32_t roundsPlayed = 0;
    uint32_t roundsWon = 0;

    struct Strategy
    {
        Qbead::BlochVector a[2];
        Qbead::BlochVector b[2];
    };

    struct CorrelationDiag
    {
        float x_corr, y_corr, z_corr;
    };
    const CorrelationDiag PHI_PLUS{1, -1, 1}, PHI_MINUS{-1, 1, 1}, PSI_PLUS{1, 1, -1}, PSI_MINUS{-1, -1, -1};

    float qE[2][2]; // quantum correlations (debug, method B)
    float qP[2][2]; // quantum win probability per (x,y)
    float qLimit;   // long-run quantum win rate

    Strategy strategy = {};
}

#endif // CHSH_STAGE_H