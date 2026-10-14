#include <Qbead.h>
#include <internal/QbeadUtils.h>
// Todo

// Could also, in lesson-plan
// have one spiral sketch to showcase 62 actual runs and how there is variance
// this is then different from the Prob mode

// Bonus: best response. Given Alice's two axes with angle θθ between them, Bob's best choice is b0∝T(a0+a1)b0​∝T(a0​+a1​) and b1∝T(a0−a1)b1​∝T(a0​−a1​).
// This gives Smax⁡=2(cos⁡θ2+sin⁡θ2)Smax​=2(cos2θ​+sin2θ​), which peaks at θ=90∘θ=90∘. 
// You could use this as a "hint" feature.

// What is corr_E?
// E is the average of the product of the two ±1 outcomes: E=P(A=B)−P(A≠B)E=P(A=B)−P(A=B).
// It is +1 if the outcomes always agree, −1 if they always disagree, and 0 if they are independent. 
// The agreement probability is P(A=B)=12(1+E)P(A=B)=21​(1+E). For Φ+Φ+ and axes in the x–z plane,
// EE is just the cosine of the angle between the two axes. 
// For example, at 45° it is 0.707, so they agree 85.4% of the time.

//

using namespace Qbead;

Qbead::Qbead bead;

//-----------------------
constexpr uint32_t N_TOTAL = 2000;
constexpr uint8_t  BATCH   = 20;          // rounds per loop pass
uint32_t rounds = 0, qWins = 0, cWins = 0;

// TODO: If adding this to utils use inline so the seeding stays consistant across mutliple files.
// Problem, that is c++17 compile?
// Alternative:
// inline uint32_t& rngState() {
//  static uint32_t s = 0x9E3779B9u;   // one instance, shared program-wide
//  return s;
//}
//inline uint32_t rng() {
//  uint32_t& x = rngState();
//  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
//  return x;
//}
//inline void seedRng(uint32_t s) { rngState() = s ? s : 0x9E3779B9u; }
uint32_t rngState = 0x9E3779B9u;           // fixed seed = demo mode

float qE[2][2];  // quantum correlations (debug, method B)
float qP[2][2];  // quantum win probability per (x,y)
float qLimit;    // long-run quantum win rate


// Diagonal of T, differs per Bell state
struct CorrelationDiag { float x_corr, y_corr, z_corr; };

// The four locked axes
struct Strategy { BlochVector a[2]; BlochVector b[2]; }; 

const CorrelationDiag PHI_PLUS{1,-1,1}, PHI_MINUS{-1,1,1}, PSI_PLUS{1,1,-1}, PSI_MINUS{-1,-1,-1};


static inline uint32_t rng() {
  // Marsaglia's xorshift32 -> Very long non-repeating cycle: 2^32 - 1
  rngState ^= rngState << 13; rngState ^= rngState >> 17; rngState ^= rngState << 5;
  return rngState;
}
void seedRng(uint32_t s) { rngState = s ? s : 0x9E3779B9u; }   // xorshift must not be 0
// live mode: seedRng(<32 bits from the hardware RNG, optionally ^ micros()>);
// The hardware RNG is much slower per byte, which is why you use it only for the seed and not as a stream

uint32_t hwSeed() {
  uint8_t sdOn = 0;
  uint32_t s = 0;
  if (sd_softdevice_is_enabled(&sdOn) == NRF_SUCCESS && sdOn &&
      sd_rand_application_vector_get((uint8_t*)&s, sizeof(s)) == NRF_SUCCESS) {
    return s ? s : 0x9E3779B9u;
  }
  // micros near boot will be very similar each time! beware
  return (micros() ^ 0xA5A5A5A5u) | 1u;      // fallback, never 0
}
// TODO: Test. Call hwSeed() once from a sketch that never starts BLE, to confirm it falls back without resetting. Adding the header may need #include <nrf_soc.h> and <nrf_sdm.h>, depending on your core. I haven't checked this in your buil

// setup(): seedRng(hwSeed());       demo mode: seedRng(0x9E3779B9u);

// Inputs: Basis alice and Basis bob
// Computes E = a^T . T . b
static inline float corr_E(const CorrelationDiag& T, const BlochVector& a, const BlochVector& b) {
     return a.x * T.x_corr * b.x + a.y * T.y_corr * b.y + a.z * T.z_corr * b.z; 
    }

// TODO: Add to utils?
// Because of the shift I added I'm currently not using this anymore. I guess I could add a shift param with default 8?
//static inline float uniform(uint32_t randIn) {
//   return (randIn >> 8) * (1.0f / 16777216.0f); 
//}

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
// TODO
// For debugging, print E after lockIn, 
// check: S = E00+E01+E10-E11 == 8*(qLimit-0.5)
// Both are the same quantity written two ways, so a mismatch means a bug.
// TODO
// Sanity check. With PHI_PLUS and Alice (z,x)(z,x), Bob ((z+x)/2,(z−x)/2)((z+x)/2​,(z−x)/2),
// qLimit must be 0.8536 and qE must show +0.707+0.707 three times and −0.707−0.707 once.
// If it doesn't, check the sign conventions in your tracker.


void playRound(bool &qWin, bool &cWin) {           // same (x,y) for both strategies
  uint32_t r = rng();
  // Xorshift's lowest bits are its weakest. Take highest bits
  int x = (r >> 31) & 1, y = (r >> 30) & 1;
  // This gives a 22-bit uniform, with resolution of about 2.4×10−72.4×10−7
  qWin = ((r & 0x3FFFFFu) * (1.0f / 4194304.0f)) < qP[x][y]; // 22 low-ish bits
  cWin = !(x & y);                       // always answer 0
}


void showLiquidFill(float qFrac, float cFrac){
  return;
};

 // setup(): seedRng(hwSeed());       demo mode: seedRng(0x9E3779B9u);
void setup() {
  bead.begin();
  bead.setBrightness(25);
  // Test the pixels by flashing a colorful pattern to make sure they are working.
  bead.testPixels();
  //
  seedRng(hwSeed());
}

// TODO: Consistent rate per s
// Pacing. The fill speed is BATCH times your loop rate. 
// If you want it independent of loop speed, use time-based pacing, e.g. a target rounds-per-second budget computed from millis().


void loop() {
  bead.readIMU(false);                 // keep trackers alive; avoid long stalls

  if (running && rounds < N_TOTAL) {
    for (uint8_t i = 0; i < BATCH && rounds < N_TOTAL; i++) {
      bool qWin, cWin;
      playRound(qWin, cWin);
      qWins += qWin;
      cWins += cWin;
      rounds++;
    }
  }

  float qFrac = rounds ? (float)qWins / rounds : 0.f;
  float cFrac = rounds ? (float)cWins / rounds : 0.f;
  showLiquidFill(qFrac, cFrac);            // your fill code
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