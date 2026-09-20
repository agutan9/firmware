#include <Qbead.h>

Qbead::Qbead bead;

void setup() {
  bead.begin();
  bead.setBrightness(25);
  bead.testPixels();
}

void loop() {
  bead.readIMU();
  bead.clear();
  bead.setBloch_deg_smooth(bead.t_acc, bead.p_acc, color(255, 0, 255));
  bead.show();
}
