// # Tap to Measure
//

// First, let's include the Qbead library and set up a few useful data structures.
#include <internal/BlochVector.h>
#include <internal/QbeadUtils.h>
#include <Qbead.h>

using namespace Qbead;

Qbead::Qbead bead;

BlochVector current_state(90, 0);

// Prepare some colors for the visualization during the game.
uint32_t white = color(255, 255, 255);
uint32_t red = color(255, 0, 0);
uint32_t blue = color(0, 0, 255);

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
  static long last_tap = 0;
  static uint32_t tap_color = white;

  // Clear the display.
  bead.clear();
  // Read the IMU to get the current gravity direction.
  bead.readIMU(false);
  if (bead.wasTapped()){
    last_tap = millis();
    BlochVector acc_vector(bead.x_whentapped, bead.y_whentapped, bead.z_whentapped);

    float probability = pow(innerProductAbs(current_state, acc_vector),2);
    float threshold = random(0, 100)/100.0f;
    float identity_threshold = 0.9;
    Serial.print("probability: ");
    Serial.println(probability);
    Serial.print("threshold: ");
    Serial.println(threshold);
    if (probability > identity_threshold) {
      tap_color = red;
      Serial.println("red identity");
    } else if (probability < 1 - identity_threshold) {
      tap_color = blue;
      Serial.println("blue identity");
    } else if (probability > threshold) {
      current_state = acc_vector;
      tap_color = red;
      Serial.println("red random");
    } else {
      current_state = -acc_vector;
      tap_color = blue;
      Serial.println("blue random");
    }
  }

  float delta = max(min(millis() - last_tap, 2000), 0) / 2000.0;
  uint32_t color = addColor(scaleColor(1-delta, tap_color), scaleColor(delta, white));
  bead.setBloch_deg(current_state, color);

  // Show the result.
  bead.show();
}
