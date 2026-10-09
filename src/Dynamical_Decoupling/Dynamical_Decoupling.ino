// # Dynamical Decoupling
//
// This example demonstrates the dynamical decoupling of a quantum state.
// The user is presented with a target state and a current state.
// The current state is a quantum state that is subject to decoherence.
// The target state is fixed.
// The goal of the "game" is to not let the current state deviate too much from the target state.
//
// The current state experiences decoherence (i.e. it is rotated around a fixed laboratory frame).
// The method of "dynamical decoupling" is used to combat the decoherence by frequently changing the axis of decoherence.
//
// The user can tap the Qbead to reset the current state to the target state.


// First, let's include the Qbead library and set up a few useful data structures.
#include <Qbead.h>
#include <internal/QbeadUtils.h>

using namespace Qbead;

Qbead::Qbead bead;

BlochVector current_state(90, 0);
BlochVector target_state(90, 0);

// Prepare some colors for the visualization during the game.
uint32_t purple = color(255, 0, 255);
uint32_t white = color(255, 255, 255);

// ## Setup
//
// The setup function is called once when the Qbead is powered on and it is used to initialize the Qbead and set up the game.
void setup() {
  bead.begin();
  bead.setBrightness(25);
  // Test the pixels by flashing a colorful pattern to make sure they are working.
  bead.testPixels();
}

// ## Event loop
//
// The loop function is called repeatedly until the Qbead is powered off.
// It is used to read the IMU and update the current state of the game.
void loop() {
  static bool current_state_visible = true;
  static uint32_t lastprintMs = 0;

  // Clear the display.
  bead.clear();

  // Read IMU for taps
  bead.readIMU(false);

  // ### Draw the "reference" state -- the one corresponding to no decoherence.
  bead.setBloch_deg_smooth(target_state, white);

  // ### Draw the current state, as it evolves over time under the influence of decoherence.

  // The current state is only visible when the user has tapped to reveal it.
  // Each tap toggles the visibility on or off.
  if (current_state_visible) {
    bead.setBloch_deg(current_state, purple);
  }

  // Show the result.
  bead.show();

  // ### Simulate the decoherence
  //
  // The decoherence is simulated by rotating the current state around a fixed laboratory frame.
  // We happen to use the direction of gravity as reported by the IMU in this example.
  current_state.rotateAround(BlochVector(bead.x, bead.y, bead.z), 0.2);

  if (bead.wasTapped()) {
    if (current_state_visible){
      Serial.println("[INFO]{TAP} User tap: Setting current state to be invisible!");
    }
    else{
      Serial.println("[INFO]{TAP} User tap: Setting current state to be visible!");
    }
    current_state_visible = !current_state_visible;
  }

  current_state_visible = current_state_visible || (millis() < 5000);
  
  if (millis() - lastprintMs > 250){
    Serial.print(millis());
    Serial.print("  | Visible: ");
    Serial.print(current_state_visible);
    Serial.print(" | Angle: ");
    Serial.print(current_state.phi);
    Serial.println();
    lastprintMs = millis();
  }

}
