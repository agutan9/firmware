#ifndef QBEAD_BLE_H
#define QBEAD_BLE_H

#include <bluefruit.h>
#include "QbeadUtils.h"

namespace BLEManager
{
  struct DataPacket
  {
    uint8_t type; // this could be defined to refer to different operations
    uint8_t value;
  };

  enum class Role
  {
    Peripheral,
    Central
  };

  class BLEManager
  {
  private:
    static BLEManager *instance;
    DataPacket lastPacket;

  public:
    BLEManager()
        : qBeadClientService(Qbead::QB_UUID_SERVICE),
          qBeadDataClient(Qbead::QB_UUID_DATA_CHAR),
          bleservice(Qbead::QB_UUID_SERVICE),
          qBeadDataChar(Qbead::QB_UUID_DATA_CHAR)
    {
    }

    BLEClientService qBeadClientService;
    BLEClientCharacteristic qBeadDataClient;

    BLEService bleservice;
    BLECharacteristic qBeadDataChar;

    bool takePacket(DataPacket &packet)
    {
      if (lastPacket.type == 0)
      {
        return false;
      }

      packet = lastPacket;
      lastPacket = {0, 0};
      return true;
    }

    void beginPeripheral()
    {
      instance = this;
      Bluefruit.begin(QB_MAX_PRPH_CONNECTION, 0);
      Bluefruit.setName("Qbead Peripheral");

      bleservice.begin();

      qBeadDataChar.setProperties(
          CHR_PROPS_READ |
          CHR_PROPS_NOTIFY);

      qBeadDataChar.setPermission(
          SECMODE_OPEN,
          SECMODE_NO_ACCESS);

      qBeadDataChar.setUserDescriptor("Qbead data");
      qBeadDataChar.setFixedLen(sizeof(DataPacket));
      qBeadDataChar.begin();

      DataPacket initial = {0, 0};
      qBeadDataChar.write(&initial, sizeof(initial));

      startBLEadv();
    }

    void startBLEadv(void)
    {
      Serial.println("[INFO]{BLE} Start advertising...");
      // Advertising packet
      Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
      Bluefruit.Advertising.addTxPower();

      // Include HRM Service UUID
      Bluefruit.Advertising.addService(bleservice);

      // Secondary Scan Response packet (optional)
      // Since there is no room for 'Name' in Advertising packet
      Bluefruit.ScanResponse.addName();

      /* Start Advertising
       * - Enable auto advertising if disconnected
       * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
       * - Timeout for fast mode is 30 seconds
       * - Start(timeout) with timeout = 0 will advertise forever (until connected)
       *
       * For recommended advertising interval
       * https://developer.apple.com/library/content/qa/qa1931/_index.html
       */
      Bluefruit.Advertising.restartOnDisconnect(true);
      Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
      Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
      Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
    }

    void beginCentral()
    {
      instance = this;
      Serial.begin(9600);
      Bluefruit.begin(0, 1);
      Bluefruit.setName("Qbead Central");

      qBeadClientService.begin();

      qBeadDataClient.setNotifyCallback(data_callback);
      qBeadDataClient.begin();

      Bluefruit.Central.setConnectCallback(connect_callback);
      Bluefruit.Central.setDisconnectCallback(disconnect_callback);

      startBLEScan();
    }

    void startBLEScan(void)
    {

      // setup the scanner
      /* Start Central Scanning
       * - Enable auto scan if disconnected
       * - Interval = 100 ms, window = 80 ms
       * - Filter only accept bleuart service in advertising
       * - Don't use active scan (used to retrieve the optional scan response adv packet)
       * - Start(0) = will scan forever since no timeout is given
       */
      Bluefruit.Scanner.setRxCallback(scan_callback);
      Bluefruit.Scanner.restartOnDisconnect(true);
      Bluefruit.Scanner.setInterval(160, 80);               // in units of 0.625 ms
      Bluefruit.Scanner.filterUuid(Qbead::QB_UUID_SERVICE); // this allows us to select a specific service
      Bluefruit.Scanner.useActiveScan(false);               // Don't request scan response data
      Bluefruit.Scanner.start(0);                           // 0 = Don't stop scanning after n seconds
    }

    static void scan_callback(ble_gap_evt_adv_report_t *report)
    {
      // Since we configure the scanner with filterUuid()
      // Scan callback only invoked for device with qbead service advertised
      // Connect to the device with qbead service in advertising packet
      Serial.println("Found Device!");
      // Stop scanning
      Bluefruit.Scanner.stop();

      Bluefruit.Central.connect(report);
    }

    static void connect_callback(uint16_t conn_handle)
    {
      if (instance == nullptr)
        return;
      Serial.println("Connected!");
      // should not be required since we filter
      if (!instance->qBeadClientService.discover(conn_handle))
      {
        Serial.println("Service NOT found");
        Bluefruit.disconnect(conn_handle);
        return;
      }

      Serial.println("Service found");

      if (!instance->qBeadDataClient.discover())
      {
        Serial.println("Characteristic NOT found");
        Bluefruit.disconnect(conn_handle);
        return;
      }

      Serial.println("Characteristic found");

      // Enable notifications if supported
      if (instance->qBeadDataClient.properties() & CHR_PROPS_NOTIFY)
      {
        instance->qBeadDataClient.enableNotify();
        Serial.println("Notifications enabled");
      }
    }

    static void data_callback(
        BLEClientCharacteristic *chr,
        uint8_t *data,
        uint16_t len)
    {
      if (instance == nullptr || len != sizeof(DataPacket))
      {
        Serial.println("Unexpected packet length");
        return;
      }

      DataPacket packet;
      memcpy(&packet, data, sizeof(packet));
      instance->lastPacket = packet;

      Serial.print("type = ");
      Serial.print(packet.type);
      Serial.print(", value = ");
      Serial.println(packet.value);
    }
    static void disconnect_callback(uint16_t conn_handle, uint8_t reason)
    {
      Serial.println("Disconnected");
      if (instance != nullptr)
      {
        instance->startBLEScan(); // restart scanning
      }
    }

    void sendData(uint8_t type, uint8_t value)
    {
      DataPacket packet = {type, value};

      qBeadDataChar.write(&packet, sizeof(packet));

      for (uint16_t conn = 0; conn < QB_MAX_PRPH_CONNECTION; conn++)
      {
        if (Bluefruit.connected(conn) &&
            qBeadDataChar.notifyEnabled(conn))
        {
          qBeadDataChar.notify(&packet, sizeof(packet));
        }
      }
    }

  }; // end class
  BLEManager *BLEManager::instance = nullptr;
}

#endif