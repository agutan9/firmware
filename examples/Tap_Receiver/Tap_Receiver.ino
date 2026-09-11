#include <Qbead.h>
#include <QbeadUtils.h>

using namespace Qbead;

Qbead::Qbead bead(BLEManager::Role::Dual);

uint32_t red = color(255, 0, 0);

void flash(uint32_t flashColor, uint16_t durationMs = 120)
{
  bead.clear();

  for (int i = 0; i < bead.pixels.numPixels(); i++) {
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
  if (bead.takeTapReceived()) {
    Serial.println("Remote tap: flashing red");

    flash(red);
  }
}