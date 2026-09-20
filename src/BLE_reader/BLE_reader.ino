#include <Qbead.h>

Qbead::Qbead bead;

void setup() {
  bead.begin();
  bead.setBrightness(25);
  bead.testPixels();
}

void loop() {
  bead.clear();
  bead.setBloch_deg_smooth(bead.t_ble, bead.p_ble, bead.c_ble);
  bead.show();
}
