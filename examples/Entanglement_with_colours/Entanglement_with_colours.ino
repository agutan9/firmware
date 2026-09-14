#include <Qbead.h>
#include <QbeadUtils.h>

using namespace Qbead;

Qbead::Qbead bead(BLEManager::Role::Dual);

uint32_t blue = color(0, 0, 255);
uint32_t red = color(255, 0, 0);
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
}

void asd()
{
    BLEManager::DataPacket packet = bead.takeLatestPacket();
    if (packet.type == static_cast<uint8_t>(Qbead::CommandType::SetOrchestrator))
    {
        Serial.println("Setting other party to be orchastrator");
        flash(red);

        bead.setOrchestrate(packet.value);
    }
    if (bead.wasTapped())
    {
        tap_count++;
        if (tap_count < 10)
        {
            Serial.println("tapped");
            return;
        }
        Serial.println("Setting myself to be orchastrator");
        flash(blue);

        bead.orchastrate();
    }
}

void loop()
{
    bead.readIMU(false);

    if (bead.isOrchastratorSet == 0)
    {
        Serial.println("Scanning for taps...");
        asd();
    }

    if (bead.isOrchastratorSet == 1)
    {
        flash(blue);
    }
    else if (bead.isOrchastratorSet == 2)
    {
        flash(red);
    }
    delay(500);
}