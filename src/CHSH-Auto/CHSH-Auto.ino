#include "Qbead.h"
#include "internal/QbeadUtils.h"

using namespace Qbead;

Qbead::Qbead bead;

//-----------------------
constexpr uint32_t N_TOTAL = 5000;
constexpr uint32_t RUN_MS = 10000;               // total animation time

uint32_t runStart = 0;
uint32_t rounds = 0, qWins = 0, cWins = 0;

float qE[2][2];  // quantum correlations (debug, method B)
float qP[2][2];  // quantum win probability per (x,y)
float qLimit;    // long-run quantum win rate

// Diagonal of T, differs per Bell state
struct CorrelationDiag { float x_corr, y_corr, z_corr; };

// The four locked axes
struct Strategy { BlochVector a[2]; BlochVector b[2]; }; 

const CorrelationDiag PHI_PLUS{1,-1,1}, PHI_MINUS{-1,1,1}, PSI_PLUS{1,1,-1}, PSI_MINUS{-1,-1,-1};

// Inputs: Basis alice and Basis bob
// Computes E = a^T . T . b
static inline float corr_E(const CorrelationDiag& T, const BlochVector& a, const BlochVector& b) {
     return a.x * T.x_corr * b.x + a.y * T.y_corr * b.y + a.z * T.z_corr * b.z; 
    }

// TODO: Normalize first. The comment says axes must be normalized, but nothing enforces it. Normalize the four vectors before building Strategy, or add a debug assert that ∣E∣≤1∣E∣≤1
void lockIn(const CorrelationDiag& T, const Strategy& s) {    // axes must be normalized
  float sum = 0;
  for (int x = 0; x < 2; x++) for (int y = 0; y < 2; y++) {
    float w = (x & y) ? -1.f : 1.f;
    qE[x][y] = corr_E(T, s.a[x], s.b[y]);
    qP[x][y] = 0.5f * (1.f + w * qE[x][y]);
    sum += qP[x][y];
  }
  // Long-run rate 1/4 ∑ p_{xy} = 1/2 + S/8
  qLimit = 0.25f * sum;                              // = 1/2 + S/8
}


void playRound(bool &qWin, bool &cWin) {           // same (x,y) for both strategies
  uint32_t r = updateRNG();
  // Xorshift's lowest bits are its weakest. Take highest bits
  int x = (r >> 31) & 1, y = (r >> 30) & 1;
  // This gives a 22-bit uniform, with resolution of about 2.4×10−72.4×10−7
  qWin = ((r & 0x3FFFFFu) * (1.0f / 4194304.0f)) < qP[x][y]; // 22 low-ish bits
  cWin = !(x & y);                       // always answer 0
}

uint32_t fadeStartTime = 0;

////        uint32_t fadeNow = now - fadeStartTime;
//        uint32_t halfInterval = fadeInterval / 2;
//        // fadeNow is implicitly % fadeInterval here
//        uint8_t phase = (uint8_t)((fadeNow * 255UL) / fadeInterval);
//        // Want to start at 1.0 (sin8 starts at 0.5) so need to phase shift to a cosine
//        luxLevel = Adafruit_NeoPixel::sine8(phase + 64) / 255.0f;

void testSimulation(){
  delay(2500);  
  float S = 0;
  for (uint8_t x = 0; x < 2; x++)
  {
    for (size_t y = 0; y < 2; y++)
    {
      Serial.printf("E[%u][%u]: %f\n", x, y, qE[x][y]);
      S += ((x & y) ? -1.f : 1.f) * qE[x][y];   // minus on E11
    }
  }
  Serial.printf("Win rate: %f\n", qLimit);
  float rhs = 8*(qLimit-0.5);
  Serial.println("S = 8 * (qLim - 0.5)");
  Serial.printf("%f = %f -> %s\n", S, rhs, fabs(S - rhs) < 0.0001 ? "True" : "False");
}


void setup() {
  bead.begin();
  bead.setBrightness(25);
  // Test the pixels by flashing a colorful pattern to make sure they are working.
  bead.testPixels();
  //
  lockIn(PHI_PLUS, Strategy{
  { BlochVector(0,  0),     // a0: +z
    BlochVector(90, 0) },   // a1: +x
    //BlochVector(35, 0) }, // ~78% winrate
  { BlochVector(45, 0),     // b0: (z + x)/sqrt2
    BlochVector(45, 180) }  // b1: (z - x)/sqrt2
  });
 
  // Overrule hardware seed
  //seedRNG(1909);
  bead.clear();
  testSimulation();

  runStart = millis();
  fadeStartTime = millis();
}

bool finished = false;
void loop() {
  constexpr bool isClassicalBead = false;// TODO: Better

  bead.readIMU(false);                 // keep trackers alive; avoid long stalls

  if (rounds < N_TOTAL) {
    uint32_t elapsed = millis() - runStart;
    uint32_t target = (elapsed >= RUN_MS) ? N_TOTAL
                     : (uint32_t)((uint64_t)N_TOTAL * elapsed / RUN_MS);
    while (rounds < target) {                   // catch up to the schedule
      bool qWin, cWin;
      playRound(qWin, cWin);
      qWins += qWin;
      cWins += cWin;
      rounds++;
    }
    Serial.printf("Current wins: %d\n", qWins);
  }
  else{
    if (!finished){
      Serial.printf("Wins Total: %lu\n", qWins);
      Serial.printf("Win rate: %.2f%%\n", 100.0f * qWins / N_TOTAL);
      finished = !finished;
    }    
  }
  //float qFrac = rounds ? (float)qWins / rounds : 0.f;
  //float cFrac = rounds ? (float)cWins / rounds : 0.f;
  bead.clear();
  //showLiquidFill(isClassicalBead ? cFrac : qFrac);   // each bead shows its own strategy
  bead.prMode.showDualLiquidFill(qWins, rounds - qWins, N_TOTAL);
  bead.prMode.showThresholdBand(fadeStartTime, bead.getLegPixelColor(0, 4));
  bead.show();
}


// A. Direct bernoulli Loop
//uint32_t wins = 0;
//uint32_t losses = 0;
//void methodA(){
//  for (uint32_t i = 0; i < N; ++i) 
//  {
//      int x = uniform() < 0.5f ? 0 : 1;
//      int y = uniform() < 0.5f ? 0 : 1;
//
//      bool win = uniform() < pWin[x][y];
//
//      if (win) {
//          ++wins;
//      } else {
//          ++losses;
//      }
//
//      updateLiquid(wins, losses);
//  }
//}

// Option B. If you want to show the individual outcomes of Alice or Bob

// Level 1: per-round outcomes, table instead of vectors
//bool roundWithOutcomes(int x, int y, int &A, int &B) {
//  A = (uniform() < 0.5f) ? +1 : -1;
//  B = (uniform() < 0.5f * (1.f + A * E[x][y])) ? +1 : -1;
//  return A * B == ((x & y) ? -1 : 1);
//}
// -> B. Collapse-table loop- same as above roundWithOutcomes only with a for loop around it
//float E[2][2];  // Precomputed correlations
//void methodB(){
//  for (uint32_t i = 0; i < N; ++i) {
//      int x = uniform() < 0.5f ? 0 : 1;
//      int y = uniform() < 0.5f ? 0 : 1;
//
//      int A = uniform() < 0.5f ? +1 : -1;
//
//      float pBobPlus = 0.5f * (1.0f + A * E[x][y]);
//      int B = uniform() < pBobPlus ? +1 : -1;
//
//      bool win = (A * B) == ((x & y) ? -1 : +1);
//
//      if (win) ++wins;
//      else ++losses;
//  }
//}

// ------------- Direct binomial draw. No visible win/loss streaks ----------------------- //
//inline float uniform() { return (rng() >> 8) * (1.0f / 16777216.0f); }
//
//uint32_t sampleWins(uint32_t N, float p) {         // Bin(N,p): exact
//  uint32_t w = 0;
//  for (uint32_t i = 0; i < N; i++){
//    w += (uniform() < p); 
//  }
//  return w;
//}

// QBEADS CODE BELOW
//    uint32_t now = millis();
//    if (fadeToggle)
//    {
//        uint32_t fadeNow = now - fadeStartTime;
//        uint32_t halfInterval = fadeInterval / 2;
//        // fadeNow is implicitly % fadeInterval here
//        uint8_t phase = (uint8_t)((fadeNow * 255UL) / fadeInterval);
//        // Want to start at 1.0 (sin8 starts at 0.5) so need to phase shift to a cosine
//        luxLevel = Adafruit_NeoPixel::sine8(phase + 64) / 255.0f;
//        if (!fadeFlip && fadeNow >= halfInterval)
//        {
//            fadeFlip = true; // disable further flips
//            entangledToggle = !entangledToggle;
//            lastFullToggleTime = now; // Count Interval from fade-in
//        }
//        if (fadeNow >= fadeInterval)
//        { // Reset and exit fade sub-loop
//            fadeFlip = false;
//            fadeToggle = false;
//            luxLevel = 1.0f;
//        }
//    }
//    else
//    { // toggleInterval counts from the moment of fading in
//        if (now - lastFullToggleTime >= toggleInterval)
//        {
//            fadeToggle = true;
//            fadeStartTime = now;
//        }
//    }

// TODO: Could use a hardcoded LU. Perhaps with a check if the assumed tot# of pixels is still the same(?)
// Codestyle question
// Don't think I need this?
// Can just use setLegPixelColour
//void initPixelLUT(const Qbead::Qbead &bead)
//{
//    // Trivial: Do the poles seperately
//    // DUE TO BUG OF REVERSED FIRST ORDER I HAVE EVERYTHING REVERSED NOW
//    // TODO: SET RIGHT
//
//    //// Northpole aligned with +Z unit vector
//    // pixelLUT[0] = {0.0, 0.0,  1.0};
//    //// Southpole aligned with -Z unit vector
//    // pixelLUT[6] = {0.0, 0.0, -1.0};
//    //// Non-trivial: All the unique-per-leg pixels
//
//    // Northpole aligned with -Z unit vector
//    pixelLUT[0] = {0.0, 0.0, -1.0};
//    // Southpole aligned with +Z unit vector
//    pixelLUT[6] = {0.0, 0.0, +1.0};
//    // Non-trivial: All the unique-per-leg pixels
//
//    // float band_theta = bead.theta_quant;
//    float x = 0.0f;
//    float y = 0.0f;
//    float z = 0.0f;
//
//    // TODO: BUG The first leg has reverse indexation order as all others!!!
//    // Applying a reverse order hack here for now, but ideally this gets resolved
//    // float cum_theta = 0.0f;
//    float cum_theta = bead.nsections * bead.theta_quant;
//    // Do the first leg seperately because it has the 6 pole issue
//    for (int fleg_th = 1; fleg_th < bead.nsections; ++fleg_th)
//    {
//        //
//        // cum_theta += bead.theta_quant;
//        cum_theta -= bead.theta_quant;
//#ifdef VERBOSE
//        x = Qbead::sin_deg(cum_theta);
//        y = 0.0f;
//        z = Qbead::cos_deg(cum_theta);
//        Serial.printf("Pixel ID[%d] Coords: {X:%.2f, Y:%.2f, Z:%.2f}\n", fleg_th, x, y, z);
//#endif
//        //
//        pixelLUT[fleg_th] = {
//            Qbead::sin_deg(cum_theta), // X = cos(p)sin(t)
//            0.0f,                      // Y = sin(p)sin(t)
//            Qbead::cos_deg(cum_theta)  // Z = cos(t)
//        };
//    }
//    // Now we can shift the other legs correctly
//    float band_sin_theta = 0.0f;
//    float leg_cos_phi = 0.0f;
//    float leg_sin_phi = 0.0f;
//    int shift = bead.nsections - 1;
//    int p_i = 1;
//
//    for (int ph_j = 1; ph_j < bead.nlegs; ++ph_j)
//    {
//        leg_cos_phi = Qbead::cos_deg(ph_j * bead.phi_quant);
//        leg_sin_phi = Qbead::sin_deg(ph_j * bead.phi_quant);
//        cum_theta = 0.0f;                                  // desynced from th_i count; use cum. counter
//        for (int th_i = 2; th_i <= bead.nsections; ++th_i) // +1 offset from 0 also
//        {
//            p_i = th_i + ph_j * shift;
//            //
//            cum_theta += bead.theta_quant;
//            band_sin_theta = Qbead::sin_deg(cum_theta);
//#ifdef VERBOSE
//            x = leg_cos_phi * band_sin_theta;
//            y = leg_sin_phi * band_sin_theta;
//            z = Qbead::cos_deg(cum_theta);
//            Serial.printf("Pixel ID[%d] Coords: {X:%.2f, Y:%.2f, Z:%.2f}\n", p_i, x, y, z);
//#endif
//            pixelLUT[p_i] = {
//                leg_cos_phi * band_sin_theta, // X = cos(p)sin(t)
//                leg_sin_phi * band_sin_theta, // Y = sin(p)sin(t)
//                Qbead::cos_deg(cum_theta)     // Z = cos(t)
//            };
//        }
//    }
//}
//
//uint32_t getEntangledContourColour(float geomInProd, float luxScale, bool gammaCorrect = true)
//{
//    // TODO: JUST SHOWCASE. Instead of plane geomInProd would need some kind of correlation calculation
//    uint32_t colorMapped = mapCorrelationDiscontinuous(geomInProd, luxScale);
//    return gammaCorrect ? Adafruit_NeoPixel::gamma32(colorMapped) : colorMapped;
//}
//
//uint32_t mapCorrelationDiscontinuous(float geomInProd, float luxScale)
//{
//    float mag = geomInProd;
//    if (geomInProd <= 0)
//        mag *= -1;
//    int idx = 0.0f;
//    if (mag < 0.05f)
//        idx = 4; // ~0.0 (orthogonal to axis)
//    else if (mag < 0.25f)
//        idx = 3; // ~0.13 (small angle off orthognal)
//    else if (mag < 0.7f)
//        idx = 2; // ~0.50 (45deg)
//    else if (mag < 0.95f)
//        idx = 1; // ~0.87 (smal angle off parallel)
//    else
//        idx = 0;
//    //
//    HSVBand band_HSV = correlationBlueYellowBands[idx];
//    // HSVBand band_HSV = (geomInProd >= 0.0f) ? correlationBlueVioletBands[rev_idx] : correlationVioletMagentaBands[rev_idx];
//    return Adafruit_NeoPixel::ColorHSV(band_HSV.hue, band_HSV.sat, band_HSV.val * luxScale);
//}
//void setEntangledContourBands(Qbead::Qbead &bead, const Qbead::BlochVector &arbAxis, float luxScale, bool half = false)
//{
//    float x = 0.0f;
//    float y = 0.0f;
//    float z = 0.0f;
//    float sum = 0.0f;
//
//    // TODO :Probbaly remove as it doesnt seem like it could work with this few pixels
//    const uint8_t pixelCount = half ? (((NUM_PIXELS - 2) / 2) + 2) : NUM_PIXELS;
//
//    for (int p_i = 0; p_i < pixelCount; p_i++)
//    {
//        if (half)
//        {
//            if (p_i == 0 || p_i == 6)
//                p_i++;
//        }
//
//        x = pixelLUT[p_i].x * arbAxis.x;
//        y = pixelLUT[p_i].y * arbAxis.y;
//        z = pixelLUT[p_i].z * arbAxis.z;
//        sum = x + y + z;
//#ifdef VERBOSE
//        Serial.printf("Pixel ID[%d] InProd: {X:%.2f, Y:%.2f, Z:%.2f} Sum: %.2f\n", p_i, x, y, z, sum);
//#endif
//
//        bead.pixels.setPixelColor(p_i, getEntangledContourColour(
//                                           pixelLUT[p_i].x * arbAxis.x +
//                                               pixelLUT[p_i].y * arbAxis.y +
//                                               pixelLUT[p_i].z * arbAxis.z
//                                           // in-product with pixel's basis unit vecs
//                                           ,
//                                           luxScale, gammaToggle));
//    }
//}
//
//// SECTION Testing and Utility
////  TODO: Remove and maybe add to Utils?
//void showPixelIdsOneByOne(Qbead::Qbead &bead, uint8_t waitTime, uint8_t lastID)
//{
//    // Showcases the ordering of the pixel ID's
//    int counter = 0;
//    for (int t = 0; t < lastID; ++t)
//    {
//        delay(waitTime);
//        bead.clear();
//        bead.pixels.setPixelColor(t, white);
//        bead.show();
//        Serial.print("Showing only Pixel ID: ");
//        Serial.println(counter);
//        counter++;
//    }
//}