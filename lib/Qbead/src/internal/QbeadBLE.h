/**
 * @file QbeadBLE.h
 * @brief Bluefruit BLE peripheral/central manager for Qbead-to-Qbead communication.
 *
 * Implementations live in QbeadBLE.cpp.
 */
#ifndef QBEAD_BLE_H
#define QBEAD_BLE_H

#include <bluefruit.h>
#include "QbeadUtils.h"

/**
 * @brief BLE types and the BLE manager used by the Qbead.
 */
namespace BLEManager
{
  /**
   * @brief Command identifier carried in every DataPacket.
   */
  enum class CommandType : uint8_t
  {
    None = 0,                   /**< No packet / empty slot. */
    Tap = 1,                    /**< A tap gesture occurred. */
    AddState = 2,               /**< Add a Bloch state (theta, phi). */
    ClearStates = 3,            /**< Clear all stored states. */
    PreparedVisualizations = 4, /**< Select a prepared visualisation (value). */
    Entangle = 5,               /**< Entanglement request; handled as a dedicated event. */
    Measure = 6,                /**< Measurement event. */
    ChshAddressLow = 7,         /**< CHSH game: low bytes of the sender address. */
    ChshAddressHigh = 8,        /**< CHSH game: high bytes of the sender address. */
    ChshRoleReady = 9,          /**< CHSH game: role negotiated and ready. */
    ChshAxisLocked = 10,        /**< CHSH game: measurement axis locked in. */
    ChshFirstSetting = 11,      /**< CHSH game: first measurement setting. */
    ChshResult = 12,            /**< CHSH game: round result. */
    ChshResultAck = 13,         /**< CHSH game: result acknowledged. */
    ChshSimulating = 14,        /**< CHSH game: simulation in progress. */
  };

  /**
   * @brief Fixed-size packet exchanged over the Qbead data characteristic.
   */
  struct DataPacket
  {
    CommandType type; /**< Command identifier. */
    uint32_t value;   /**< Command-specific payload. */
    uint32_t theta;   /**< Command-specific polar angle (typically degrees). */
    uint32_t phi;     /**< Command-specific azimuthal angle (typically degrees). */
  };

  /**
   * @brief Owns the Qbead BLE service, characteristics and client-side counterparts.
   *
   * Supports peripheral-only, central-only and dual-role operation. Bluefruit callbacks
   * carry no context pointer, so the active manager is stored in a static @c instance,
   * set by the begin*() methods. Only one BLEManager should be active at a time.
   *
   * Incoming packets are kept in a single-slot mailbox (takePacket()); entanglement
   * requests are additionally latched with their arrival time (takeEntangleRequest()).
   */
  class BLEManager
  {
  private:
    static BLEManager *instance;                          /**< Active manager used by the static callbacks. */
    DataPacket lastPacket = {CommandType::None, 0, 0, 0}; /**< Most recently received packet; None if consumed. */

    volatile bool entangleRequestPending = false;      /**< Set by data_callback() on an Entangle packet. */
    volatile uint32_t entangleRequestReceivedAtMs = 0; /**< millis() at which the pending Entangle packet arrived. */

    /**
     * @brief Starts the Qbead service and configures the data characteristic (read + notify).
     */
    void setupPeripheral();

    /**
     * @brief Starts the client service/characteristic and registers the central callbacks.
     */
    void setupCentral();

  public:
    /**
     * @brief Constructs all services and characteristics with the Qbead UUIDs.
     *
     * Nothing is started until one of the begin*() methods is called.
     */
    BLEManager();

    BLEClientService qBeadClientService;     /**< Remote Qbead service (central role). */
    BLEService bleservice;                   /**< Local Qbead service (peripheral role). */
    BLEClientCharacteristic qBeadDataClient; /**< Remote data characteristic (central role). */
    BLECharacteristic qBeadDataChar;         /**< Local data characteristic (peripheral role). */

    BLEClientCharacteristic qBeadTapClient; /**< Remote tap characteristic. */
    BLECharacteristic qBeadTapChar;         /**< Local tap characteristic. */

    BLEClientCharacteristic qBeadCharAccClient; /**< Remote accelerometer characteristic. */
    BLECharacteristic qBeadCharAccChar;         /**< Local accelerometer characteristic. */

    BLEClientCharacteristic qBeadSphCharClient; /**< Remote spherical-coordinates characteristic. */
    BLECharacteristic qBeadSphCharChar;         /**< Local spherical-coordinates characteristic. */

    BLEClientCharacteristic qBeadColCharClient; /**< Remote colour characteristic. */
    BLECharacteristic qBeadColCharChar;         /**< Local colour characteristic. */

    /**
     * @brief Reads the local BLE MAC address.
     * @param[out] localAddress Six-byte buffer receiving the address.
     */
    void getAddr(uint8_t localAddress[6]);

    /**
     * @brief Takes the most recently received packet, if any, and clears the mailbox.
     * @param[out] packet Receives the packet; unchanged if none is pending.
     * @return True if a packet was available.
     * @note Only the latest packet is kept; earlier unread packets are overwritten.
     */
    bool takePacket(DataPacket &packet);

    /**
     * @brief Takes a pending entanglement request, if any.
     * @param[out] receivedAtMs millis() timestamp at which the request arrived.
     * @return True if a request was pending; the pending flag is then cleared.
     */
    bool takeEntangleRequest(uint32_t &receivedAtMs);

    /**
     * @brief Starts BLE in peripheral-only mode ("Qbead Peripheral") and begins advertising.
     */
    void beginPeripheral();

    /**
     * @brief Starts advertising the Qbead service, restarting automatically on disconnect.
     *
     * Fast interval 20 ms for 30 s, then slow interval 152.5 ms; advertises forever.
     */
    void startBLEadv(void);

    /**
     * @brief Starts BLE in central-only mode ("Qbead Central") and begins scanning.
     * @note Also calls Serial.begin(9600).
     */
    void beginCentral();

    /**
     * @brief Starts BLE in dual-role mode ("Qbead Dual"): advertises and scans at the same time.
     */
    void beginDualRole();

    /**
     * @brief Starts scanning for devices advertising the Qbead service.
     *
     * Interval 100 ms, window 50 ms, passive scan, restarts on disconnect, scans forever.
     */
    void startBLEScan(void);

    /**
     * @brief Scanner callback: stops scanning and connects to the reported Qbead.
     * @param report Advertising report of the discovered device.
     */
    static void scan_callback(ble_gap_evt_adv_report_t *report);

    /**
     * @brief Central connect callback: discovers the service and data characteristic and enables notifications.
     *
     * Disconnects if the service or characteristic is not found.
     *
     * @param conn_handle Connection handle.
     */
    static void connect_callback(uint16_t conn_handle);

    /**
     * @brief Notification callback: stores the incoming packet and latches entanglement requests.
     * @param chr  Characteristic that notified.
     * @param data Packet bytes.
     * @param len  Length of @p data; packets not equal to sizeof(DataPacket) are dropped.
     */
    static void data_callback(
        BLEClientCharacteristic *chr,
        uint8_t *data,
        uint16_t len);

    /**
     * @brief Central disconnect callback: restarts scanning.
     * @param conn_handle Connection handle.
     * @param reason      HCI disconnect reason.
     */
    static void disconnect_callback(uint16_t conn_handle, uint8_t reason);

    /**
     * @brief Writes a packet to the local data characteristic and notifies subscribed centrals.
     * @param type  Command identifier.
     * @param value Command-specific payload.
     * @param theta Command-specific polar angle.
     * @param phi   Command-specific azimuthal angle.
     */
    void sendData(CommandType type, uint32_t value, uint32_t theta,
                  uint32_t phi);

  }; // end class
}

#endif // QBEAD_BLE_H
