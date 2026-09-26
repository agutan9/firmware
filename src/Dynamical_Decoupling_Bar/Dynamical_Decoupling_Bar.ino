// # Dynamical Decoupling (Bar Visualization)
//
// This example demonstrates dynamical decoupling using a bar visualization.
// Instead of tracking a single quantum state vector, we track the "spread"
// that many such vectors might have accumulated over time.
//
// The spread increases based on the device orientation (inner product between
// the zero axis and gravity). A bar is drawn around the equator showing the
// current spread - the wider the bar, the more decoherence has occurred.
//
// The user can tap the Qbead to reset the spread back to zero.


// First, let's include the Qbead library and set up a few useful data structures.
#include <Qbead.h>

Qbead::Qbead bead;

// The zero axis is the fixed axis around which decoherence rotates.
Qbead::BlochVector zero_axis(0, 0);

// Track the spread of the quantum state ensemble.
float spread = 0;

// The section (theta level) at which to draw the bar.
const int bar_section = 3;

// Prepare some colors for the visualization during the game.
uint32_t purple = Qbead::color(255, 0, 255);
uint32_t white = Qbead::color(255, 255, 255);

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
// It is used to read the IMU and update the spread of the ensemble.
void loop() {
  static bool bar_visible = true;

  // Read the IMU to get the current gravity direction.
  bead.readIMU(false);

  // Clear the display.
  bead.clear();

  // ### Draw the bar showing the current spread.

  // The bar is only visible when the user has tapped to reveal it.
  // Each tap toggles the visibility on or off.
  if (bar_visible) {
    int spread_int = int(abs(spread));
    float spread_frac = abs(spread) - spread_int;
    // Draw a bar from -spread_int to +spread_int around the equator.
    for (int leg = -spread_int; leg <= spread_int; leg++) {
      bead.setLegPixelColor(leg, bar_section, purple);
    }
    // Partially light the outermost LEDs based on the fractional part of spread.
    if (spread_int < QB_NLEGS / 2) {
      uint32_t edge_color = Qbead::scaleColorQuad(spread_frac, purple);
      bead.setLegPixelColor(spread_int + 1, bar_section, edge_color);
      bead.setLegPixelColor(-(spread_int + 1), bar_section, edge_color);
    }
  }

  // Show the result.
  bead.show();

  // ### Simulate the decoherence
  //
  // The spread increases based on the inner product between the zero axis and gravity.
  // This represents how the ensemble of quantum states would spread out over time.
  float spread_rate = 0.004 * innerProductGeom(zero_axis, Qbead::BlochVector(bead.x, bead.y, bead.z));
  spread += spread_rate;

  // Keep spread within bounds of plus-or-minus half the number of legs.
  float max_spread = QB_NLEGS / 2.0;
  spread = constrain(spread, -max_spread, max_spread);

  // ### Check for taps
  //
  // If the user taps the Qbead, toggle the visibility and reset the spread.
  if (bead.wasTapped()) {
    Serial.println("TAP");
    bar_visible = !bar_visible;
  }

  bar_visible = bar_visible || (millis() < 5000);

  Serial.print(millis());
  Serial.print("  | Visible: ");
  Serial.print(bar_visible);
  Serial.print(" | Spread: ");
  Serial.print(spread);
  Serial.print(" | Rate: ");
  Serial.println(spread_rate);
}
