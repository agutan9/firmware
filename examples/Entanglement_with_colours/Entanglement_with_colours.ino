#include <Qbead.h>
#include <QbeadUtils.h>

using namespace Qbead;

Qbead::Qbead bead;
BlochVector up(0, 0);
BlochVector down(180, 0);
int c_tap = 0;

uint32_t blue = color(0, 0, 255);
uint32_t red = color(255, 0, 0);
uint32_t white = color(255, 255, 255);

int tap_count = 0;

void flash(uint32_t flashColor, uint16_t durationMs = 120)
{
    bead.clear();

    for (int i = 0; i < bead.pixels.numPixels(); i++)
    {
        bead.pixels.setPixelColor(i, flashColor);
    }

    bead.show();
    delay(durationMs);

    bead.clear();
    bead.show();
}

void setup()
{
    bead.begin();
    bead.setBrightness(25);
    bead.testPixels();
    bead.clear();
}

void loop()
{
    bead.readIMU(false);
    BLEManager::DataPacket packet = bead.takeLatestPacket();
    if (packet.type == BLEManager::CommandType::AddState)
    {
        BlochVector newState(packet.theta, packet.phi);
        bead.setBloch_deg(newState, packet.value);
    }
    else if (packet.type == BLEManager::CommandType::ClearStates)
    {
        c_tap = 0;
        bead.innerStateCount = 0;
        bead.clear();
    }
    else if (bead.wasTapped())
    {
        c_tap++;
        Serial.println(c_tap);
        if (c_tap >= 5 && !bead.hasState(up))
        {
            Serial.println("Setting spin up");
            bead.addState(up);
            bead.setBloch_deg(up, blue);
            bead.ble.sendData(BLEManager::CommandType::AddState, blue, up.theta, up.phi);
        }
        else if (c_tap >= 10 && !bead.hasState(down))
        {
            Serial.println("Setting spin down");
            bead.addState(down);
            bead.setBloch_deg(down, red);
            bead.ble.sendData(BLEManager::CommandType::AddState, red, down.theta, down.phi);
        }
        else if (c_tap > 20)
        {
            c_tap = 0;
            bead.innerStateCount = 0;
            bead.ble.sendData(BLEManager::CommandType::ClearStates, 0, 0, 0);
            bead.clear();
        }
    }
    bead.show();
}