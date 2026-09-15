#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>
#include <bluefruit.h>
#include <Qbead.h>

const int numPixels;

Qbead::Qbead bead;

struct PixelAxis;
// TODO IF ALLOWED TO HARDCODE MAKE static constexpr
std::vector<PixelAxis> pixelLUT;

void calcPixelUnitVecs();
void axisBands(Qbead);

// TODO: Why are we using const uint8_t and not static constexpr for some things? Does it matter performance-wise
void setup() {
  Serial.begin(9600);
  // Skip bead.begin() entirely for now. This is just for visualising the leds
  bead.pixels.begin();
  bead.pixels.setBrightness(50);
  // Dynamically set LUT size based on device specs and fill it
  numPixels = bead.pixels.numPixels();
  pixelLUT.resize(numPixels);
  initPixelLUT(bead);


}

void loop() {
      // TODO: CHECK IF 0 and 6 are not acidentally swapped
    // TODO: DO THIS BY GOING THROUGH ALL LEDS WITH PIXEL ON OFF WITH DELAY
  for (int i = 0; i < numPixels; i++) {
    bead.pixels.setPixelColor(i, bead.pixels.Color(LVL, LVL, LVL));
  }
  bead.pixels.show();
  Serial.println("Looping..")
  bead.pixels.show();
  delay(5000);
}

struct PixelAxis {
    float x, y, z;
};


// TODO: Could use a hardcoded LU. Perhaps with a check if the assumed tot# of pixels is still the same(?)
// Codestyle question 
void initPixelLUT(const Qbead &bead)
{
    // Trivial: Do the poles seperately
    // Northpole aligned with +Z unit vector
    pixelLUT[0] = {0.0, 0.0,  1.0}
    // Southpole aligned with -Z unit vector
    pixelLUT[6] = {0.0, 0.0, -1.0}
    // Non-trivial: All the unique-per-leg pixels
    float band_theta = bead.theta_quant;
    int p_i = 1; // skip pole at 0
    for (int th_i = 0; th_i < bead.nsections - 1; th_i++)
    {
        float band_cost = cos_deg(band_theta);
        float band_sint = sin_deg(band_theta);
        float cum_phi = 0;
        for (int ph_j = 0; ph_j < bead.nlegs; ph_j++)
        {
            pixelLUT[p_i] = {
                cos_deg(cum_phi) * band_sint,// X = cos(p)sin(t)
                sin_deg(cum_phi) * band_sint,// Y = sin(p)sin(t)
                band_cost                    // Z = cos(theta)
            }
            // Go to the next leg's pixel at same height
            cum_phi += bead.phi_quant;
        }
        band_theta += bead.theta_quant;
        // Need to skip  p_i = 6 as it's a pole
        p_i = ++p_i == 6 ? p_i++ : p_i; 
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

