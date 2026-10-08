#include <Qbead.h>
#include <QbeadUtils.h>

using namespace Qbead;

Qbead::Qbead bead(BLEManager::Role::Dual);

uint32_t blue = color(0, 0, 255);
uint32_t red = color(255, 0, 0);

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
}

void loop()
{
    bead.readIMU(false);

    if (bead.wasTapped())
    {
        Serial.println("Tap detected: notifying central");

        flash(blue);

        // type 1 = tap event, value 1 = tap occurred
        bead.ble.sendData(1, 1);
    }
    if (bead.takeTapReceived())
    {
        Serial.println("Remote tap: flashing red");

        flash(red);
    }
}