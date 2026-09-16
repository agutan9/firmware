#include <Qbead.h>

Qbead::Qbead bead;

void setup() {
  Serial.begin(9600);
  delay(100);
  bead.pixels.begin();
  bead.pixels.clear();
  bead.pixels.show();
  Serial.println("Entering Sleep Mode... ZZZzzz");
  delay(100);
  sd_power_system_off();
}

void loop() {
}