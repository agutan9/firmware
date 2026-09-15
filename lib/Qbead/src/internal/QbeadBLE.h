#ifndef QBEAD_BLE_H
#define QBEAD_BLE_H

#include <bluefruit.h>

namespace Qbead
{
    void connect_callback(uint16_t conn_handle)
    {
        // Get the reference to current connection
        BLEConnection *connection = Bluefruit.Connection(conn_handle);

        char central_name[32] = {0};
        connection->getPeerName(central_name, sizeof(central_name));

        Serial.print("[INFO]{BLE} Connected to "); // TODO take care of cases where Serial is not available
        Serial.println(central_name);
    }
}

#endif