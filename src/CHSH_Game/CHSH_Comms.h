#ifndef CHSH_COMMS_H
#define CHSH_COMMS_H

#include <Arduino.h>
#include <Qbead.h>
#include <internal/QbeadUtils.h>
#include <math.h>

#include "CHSH_Utils.h"

using namespace ChshStage;

namespace ChshComms
{
    using Command = BLEManager::CommandType;
    using Packet = BLEManager::DataPacket;

    // -------------------------------------------------------------------------
    // Packet transmission
    // -------------------------------------------------------------------------

    inline void sendMessage(
        Command type,
        uint32_t value,
        Qbead::Qbead &bead,
        uint32_t theta,
        uint32_t phi)
    {
        bead.ble.sendData(
            type,
            value,
            theta,
            phi);
    }

    inline void sendAxisMessage(
        Command type,
        uint32_t value,
        const Qbead::Axis &axis,
        Qbead::Qbead &bead)
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

        ChshComms::sendMessage(
            type,
            value,
            bead,
            thetaInteger,
            phiInteger);
    }

    inline bool axisFromPacket(
        const Packet &packet,
        Qbead::Axis &axis)
    {
        if (packet.theta > 180 || packet.phi >= 360)
            return false;

        axis = Qbead::axisFromDegrees(
            (float)packet.theta,
            (float)packet.phi);

        return true;
    }

    // -------------------------------------------------------------------------
    // Bluetooth address helpers
    // -------------------------------------------------------------------------

    inline uint32_t packAddressPart(
        const uint8_t *address)
    {
        return (uint32_t)address[0] | ((uint32_t)address[1] << 8) | ((uint32_t)address[2] << 16);
    }

    inline void unpackAddressPart(
        uint32_t value,
        uint8_t *address)
    {
        address[0] =
            (uint8_t)(value & 0xFFUL);

        address[1] =
            (uint8_t)((value >> 8) & 0xFFUL);

        address[2] =
            (uint8_t)((value >> 16) & 0xFFUL);
    }

    inline void printAddress(
        const uint8_t *address)
    {
        for (int i = 5; i >= 0; --i)
        {
            if (address[i] < 16)
                Serial.print('0');

            Serial.print(address[i], HEX);

            if (i > 0)
                Serial.print(':');
        }
    }

    // -------------------------------------------------------------------------
    // Automatic role assignment
    // -------------------------------------------------------------------------

    inline void tryAssignRole()
    {
        if (roleAssigned || remoteAddressParts != 0x03)
            return;

        int comparison = 0;

        for (int i = 5; i >= 0; --i)
        {
            if (localAddress[i] < remoteAddress[i])
            {
                comparison = -1;
                break;
            }

            if (localAddress[i] > remoteAddress[i])
            {
                comparison = 1;
                break;
            }
        }

        if (comparison == 0)
        {
            Serial.println(
                "[ROLE] Identical addresses; cannot assign roles.");
            return;
        }

        BEAD_ID = comparison < 0 ? 0 : 1;
        roleAssigned = true;

        Serial.print("[ROLE] Remote address: ");
        ChshComms::printAddress(remoteAddress);
        Serial.println();

        Serial.print("[ROLE] Assigned ID: ");
        Serial.println(BEAD_ID);

        Serial.println(
            BEAD_ID == 0
                ? "[ROLE] Alice / simulation coordinator."
                : "[ROLE] Bob.");
    }

    inline bool handleIdentityPacket(const Packet &packet)
    {
        switch (packet.type)
        {
        case Command::ChshAddressLow:
        {
            if (!roleAssigned)
            {
                ChshComms::unpackAddressPart(
                    packet.value,
                    &remoteAddress[0]);

                remoteAddressParts |= 0x01;
                ChshComms::tryAssignRole();
            }

            return true;
        }

        case Command::ChshAddressHigh:
        {
            if (!roleAssigned)
            {
                ChshComms::unpackAddressPart(
                    packet.value,
                    &remoteAddress[3]);

                remoteAddressParts |= 0x02;
                ChshComms::tryAssignRole();
            }

            return true;
        }

        case Command::ChshRoleReady:
        {
            if (
                roleAssigned &&
                packet.value == (uint32_t)(BEAD_ID ^ 1U))
            {
                if (!peerRoleReady)
                    Serial.println("[ROLE] Both beads ready.");

                peerRoleReady = true;
            }

            return true;
        }

        default:
            return false;
        }
    }

    inline void serviceIdentityExchange(Qbead::Qbead &bead)
    {
        const uint32_t now = millis();

        if ((uint32_t)(now - lastIdentityTxMs) < IDENTITY_TX_MS)
            return;

        lastIdentityTxMs = now;

        if (!peerRoleReady)
        {
            switch (identityTxSlot)
            {
            case 0:
                ChshComms::sendMessage(
                    Command::ChshAddressLow,
                    ChshComms::packAddressPart(
                        &localAddress[0]),
                    bead);
                break;

            case 1:
                ChshComms::sendMessage(
                    Command::ChshAddressHigh,
                    ChshComms::packAddressPart(
                        &localAddress[3]),
                    bead);
                break;

            case 2:
                if (roleAssigned)
                {
                    ChshComms::sendMessage(
                        Command::ChshRoleReady,
                        BEAD_ID,
                        bead);
                }
                break;
            }

            identityTxSlot = (identityTxSlot + 1) % 3;
        }
        else if (
            stage == ChshStage::Stage::Idle ||
            stage == ChshStage::Stage::Collapsed)
        {
            ChshComms::sendMessage(
                Command::ChshRoleReady,
                BEAD_ID,
                bead);
        }
    }

    // -------------------------------------------------------------------------
    // Packet reception
    // -------------------------------------------------------------------------

    inline void handlePacket(
        const Packet &packet,
        Qbead::Qbead &bead)
    {
        if (ChshComms::handleIdentityPacket(packet))
            return;

        if (!roleAssigned)
            return;

        switch (packet.type)
        {
        case Command::ChshAxisLocked:
        {
            // In this design, only Alice needs Bob's lock packet.
            if (
                BEAD_ID != 0 ||
                stage != ChshStage::Stage::Choosing ||
                remoteLocked)
            {
                return;
            }

            // Only bits 0 and 1 are defined.
            if (packet.value > 3)
                return;

            const uint8_t senderRole =
                (uint8_t)((packet.value >> 1) & 1U);

            if (senderRole != 1)
                return;

            Qbead::Axis decoded;

            if (!ChshComms::axisFromPacket(packet, decoded))
                return;

            remoteAxis = decoded;
            remoteInput =
                (uint8_t)(packet.value & 1U);

            remoteLocked = true;

            Serial.println("[LOCK] Bob's setting received.");

            if (!firstChosen)
                ChshUtils::chooseFirst(1, bead);

            return;
        }

        case Command::ChshFirstSetting:
        {
            if (
                BEAD_ID != 1 ||
                stage != ChshStage::Stage::Choosing ||
                firstChosen)
            {
                return;
            }

            if (packet.value > 3)
                return;

            Qbead::Axis decoded;

            if (!ChshComms::axisFromPacket(packet, decoded))
                return;

            const uint8_t announcedInput =
                (uint8_t)(packet.value & 1U);

            const uint8_t announcedRole =
                (uint8_t)((packet.value >> 1) & 1U);

            // If Bob is first, he must already have locked.
            if (announcedRole == BEAD_ID && !localLocked)
                return;

            firstChosen = true;
            firstRole = announcedRole;

            // Store flags only; angles are carried separately.
            firstMessage = packet.value;

            if (firstRole != BEAD_ID)
            {
                remoteAxis = decoded;
                remoteInput = announcedInput;
                remoteLocked = true;
            }

            Serial.print("[FIRST] First accepted bead ID: ");
            Serial.println(firstRole);

            ChshUtils::updateSecondTarget(bead);

            return;
        }

        case Command::ChshResult:
        {
            if (BEAD_ID != 1 || packet.value > 0x0FUL)
                return;

            if (
                stage == ChshStage::Stage::Choosing &&
                localLocked)
            {
                const uint8_t a =
                    packet.value & 1U;

                const uint8_t b =
                    (packet.value >> 1) & 1U;

                const uint8_t x =
                    (packet.value >> 2) & 1U;

                const uint8_t y =
                    (packet.value >> 3) & 1U;

                if (y != localInput)
                    return;

                resultMessage = packet.value;

                ChshUtils::recordResult(
                    x,
                    y,
                    a,
                    b,
                    roundsPlayed,
                    roundsWon);

                ChshUtils::setCollapsedState(
                    b,
                    bead,
                    stage);

                ChshComms::sendMessage(
                    Command::ChshResultAck,
                    resultMessage,
                    bead);
            }
            else if (
                stage == ChshStage::Stage::Collapsed &&
                packet.value == resultMessage)
            {
                // Re-acknowledge without counting again.
                ChshComms::sendMessage(
                    Command::ChshResultAck,
                    resultMessage,
                    bead);
            }

            return;
        }

        case Command::ChshResultAck:
        {
            if (
                BEAD_ID == 0 &&
                stage == ChshStage::Stage::Collapsed &&
                resultPending &&
                packet.value == resultMessage)
            {
                resultPending = false;
            }

            return;
        }

        default:
            // Leave unrelated command types alone.
            return;
        }
    }

    // -------------------------------------------------------------------------
    // Retries
    // -------------------------------------------------------------------------

    inline void serviceResends(Qbead::Qbead &bead)
    {
        const uint32_t now = millis();

        if ((uint32_t)(now - lastSendMs) < RESEND_MS)
            return;

        if (stage == ChshStage::Stage::Choosing)
        {
            if (BEAD_ID == 0 && firstChosen)
            {
                const Qbead::Axis &firstAxis =
                    firstRole == BEAD_ID
                        ? localAxis
                        : remoteAxis;

                ChshComms::sendAxisMessage(
                    Command::ChshFirstSetting,
                    firstMessage,
                    firstAxis,
                    bead);

                lastSendMs = now;
            }
            else if (BEAD_ID == 1 && localLocked)
            {
                ChshUtils::sendLocalLock(bead);
            }
        }
        else if (
            BEAD_ID == 0 &&
            stage == ChshStage::Stage::Collapsed &&
            resultPending)
        {
            ChshComms::sendMessage(
                Command::ChshResult,
                resultMessage,
                bead);

            lastSendMs = now;
        }
    }
}

#endif // CHSH_COMMS_H