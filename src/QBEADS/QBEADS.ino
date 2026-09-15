#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>
#include <bluefruit.h>
#include <Qbead.h>

Qbead::Qbead bead;

struct PixelAxis;
std::vector<PixelAxis> pixelLUT;

void calcPixelUnitVecs();
void axisBands(Qbead);

// TODO: Why are we using const uint8_t and not static constexpr for some things? Does it matter performance-wise
void setup() {
  Serial.begin(9600);
  // Skip bead.begin() entirely for now. This is just for visualising the leds
  bead.pixels.begin();
  bead.pixels.setBrightness(50);
  // Dynamically set LUT size based on device specs
  const int numPixels = bead.
  PixelAxis pixelLUT

  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, bead.pixels.Color(LVL, LVL, LVL));
  }
  bead.pixels.show();
}

void loop() {
  //// Re-assert periodically in case of any driver hiccup; keeps draw constant.
  //for (int i = 0; i < bead.pixels.numPixels(); i++) {
  //  bead.pixels.setPixelColor(i, bead.pixels.Color(LVL, LVL, LVL));
  //}
  Serial.println("Looping..")
  bead.pixels.show();
  delay(5000);
}

struct PixelAxis {
    float x, y, z;
};



// TODO IF ALLOWED TO HARDCODE MAKE static constexpr

    // z = cos(t)
    // x = cos(p)sin(t)
    // y = sin(p)sin(t)
// TODO: Could use a hardcoded LU. Perhaps with a check if the assumed tot# of pixels is still the same(?)
// Codestyle question 
void calcPixelUnitVecs(const Qbead &bead)
{
    // Do the poles seperately
    // ...
    // Do the rest
    float band_theta = bead.theta_quant;
    for (int th_i = 0; th_i < bead.nsections - 1; th_i++)
    {
        float cum_phi = 0;
        for (int ph_j = 0; ph_j < bead.nlegs; ph_j++)
        {

            // Go to the next leg's pixel at same height
            cum_phi += bead.phi_quant;
        }
        band_theta += bead.theta_quant;
    }

}

void axisBands(Qbead &bead)
{ // QBEADS is visualised in discretized contour bands around the BlochV axis
    // If axis = z then it's 7 bands (inc. 2 singular pole bands)

    
    // #pixels: nlegs * (nsections - 1) + 2
    // +2 is the poles
    // sections is bands in between pixels
    // 5 pixels per leg (excl. poles)
    // except for singularityies
    // -> every contour will be 12 pixels if Z-aligned
    // -> Contours not so aligned
    // Since its inherently discretized I could also
    // calculate the color by some formula of the angle
    // Theta is half-sphere angle
    // Phi is full-sphere angle

    // Need to turn any pixel into a BlochVector / coordinate

    // acos(InnerProductGeom) is faster version of the intuitively more correct centralAngle

    

}

