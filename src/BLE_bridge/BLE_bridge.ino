#include <Qbead.h>

Qbead::Qbead bead;

void setup() {
  bead.begin();
  Serial.println("Running BLE_bridge -- the Qbead will report its IMU readings over BLE and will present itself as a \"dumb\" spherical display.");
}

void loop() {
  // read IMU and sent over BLE (without printing to Serial)
  bead.readIMU(false);

  // set Bloch Sphere Visualizer to last values received over BLE
  bead.clear();
  bead.setBloch_deg_smooth(bead.t_ble, bead.p_ble, bead.c_ble);
  bead.show();
}
