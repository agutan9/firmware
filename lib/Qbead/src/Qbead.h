#ifndef QBEAD_H
#define QBEAD_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>
#include <bluefruit.h>

#include "internal/BlochVector.h"
#include "internal/QbeadUtils.h"
#include "internal/QbeadBLE.h"

namespace Qbead
{
  // TODO
  // Dit moet allemaal niet te veel tijd gaan kosten, lower prio, maar je perplexity chat
  // over de refactor en arbitraire axis is veel info en moet even doorgespit worden
  // Als we onderscheid willen kunnen maken tussen world X and Y axis just like the gravity axis
  // wordt het complexer. Mijn idee, zet ze statisch met een callibratie stap. Meestal blijf jij
  // als gebruiker in jouw wereld statisch staan. Er zou dan een config stap moeten komen 
  // waarmee je de links/rechts x en voor/achter y zet. .. Mmmmh
  // Ik zat helemaal te denken van, detect taps om arbitrair neiuwe X en Y te zetten maar..
  // misschien is iets waarmee je kort naar blanke sphere gaat met de global X, Y en Z axis met 
  // LEDs gekleurd veel simpler en beter. Dit soort callibratie/reset doe je meestal toch al
  // voor de Poles gravity axis. Het enige wat je dan dient te doen die LEDs voor jezelf callibreren
  // als je dan echt fancy wil zijn kan je de offset daarvan opslaan en de huidige coordinaten van
  // alle in-play sphere elementen daarmee offsetten zodat de rotations die je doet voor de callibratie stap
  // geen invloed hebben op de state die je voor je had...


  // PR REVIEW: Should these stay as structs or integrate them into the Qbead class?
  /**
   * @brief Detects a deliberate shake along a cardinal global axis.
   *
   * Each update sample is split into a component along the cardinal shake axis
   * ("parallel") and a component across it ("perpendicular"), after subtracting
   * a gravity estimate.
   * A shake is reported when a positive and a negative excursion along gravity,
   * each larger than a threshold and each mostly parallel, occur within a short
   * pairing window.
   *
   * Gravity can be filtered internally or supplied by a GravityTracker.
   * Call update() exactly once per sensor sample.
   * 
   */
  //struct ShakeDetector {
  //  float gravityEstimate[3] = {0.f, 0.f, 1.f}; // TODO: default constructor
  //  bool init = false; /**< True once gravityEstimate holds valid data. Set to false to recallibrate (e.g. after a timing gap). */
  //  bool pos
//
  //  float scaxis[3]; 
  //  float scaxis_atShakeStart[3];
  //  float vPos = 0, vNeg = 0; // members: largest peak on each side
  //  uint32_t tPos = 0, tNeg = 0, tFire = 0;
  //  bool havePos = false, haveNeg = false;
//
  //  //ShakeDetector() : 
  //  // Shake is a periodic signal with
//
  //  bool update(const float r[3], float dt, uint32_t nowMs, const float* gExt = nullptr) {
  //    /* */
  //    const float INT_TRIGAX_FILTERWINDOW = 1.0f; /**<  Not used when gExt passed.  */
  //    const float THR = 0.5f; /**< Shake half-period base threshold for acceleration projected amplitude. Default total amp: 2xTHR. Modified by P2P, CONE_RATIO.*/
  //    const float P2P = 1.9f; /**< Peak-to-Peak acceleration amplitude. TODO: HOW DOES IT RELATE TO THR???*/
  //    const float CONE_RATIO = 0.45f; /**<  */
  //    const uint32_t WINDOW_MS = 300; /**< Max time period of shake signal. */
  //    const uint32_t COOLDOWN_MS = 500; /**<  */
  //    if (gExt) { memcpy(trigAx, gExt, sizeof trigAx); init = true; }
  //    else if (!init) { memcpy(trigAx, r, sizeof trigAx); init = true; }
//
  //    float gm = sqrtf(trigAx[0]*trigAx[0] + trigAx[1]*trigAx[1] + trigAx[2]*trigAx[2]);
  //    float h[3]   = {trigAx[0]/gm, trigAx[1]/gm, trigAx[2]/gm};
  //    float lin[3] = {r[0]-trigAx[0], r[1]-trigAx[1], r[2]-trigAx[2]};
  //    float par  = lin[0]*h[0] + lin[1]*h[1] + lin[2]*h[2];
  //    float pe[3] = {lin[0]-par*h[0], lin[1]-par*h[1], lin[2]-par*h[2]};
  //    float perp = sqrtf(pe[0]*pe[0] + pe[1]*pe[1] + pe[2]*pe[2]);
//
  //    if (!gExt) {
  //      float a = dt / ((havePos || haveNeg) ? 4*INT_TRIGAX_FILTERWINDOW : INT_TRIGAX_FILTERWINDOW); // slow down mid-shake
  //      for (int i = 0; i < 3; i++) trigAx[i] += a * (r[i] - trigAx[i]);
  //    }      
  //    // Expiry windows
  //    if (havePos && nowMs - tPos > WINDOW_MS) havePos = false;
  //    if (haveNeg && nowMs - tNeg > WINDOW_MS) haveNeg = false;
  //    
  //    bool idle = !(havePos || haveNeg);
  //    if (fabsf(par) > THR && perp < CONE_RATIO * fabsf(par)) {
  //      if (idle) memcpy(trigAx_atShakeStart, trigAx, sizeof trigAx); // gravity at the first excursion
  //      if (par > 0) { if (!havePos || par > vPos) vPos = par; tPos = nowMs; havePos = true; }
  //      else         { if (!haveNeg || par < vNeg) vNeg = par; tNeg = nowMs; haveNeg = true; }
  //    }
//
  //    if (havePos && haveNeg && (vPos - vNeg) > P2P && nowMs - tFire > COOLDOWN_MS) {
  //      havePos = haveNeg = false; tFire = nowMs; return true;
  //    }
  //    return false;
  //  }
  //};

  /**
   * @brief Detects a deliberate shake along one axis (currently the vertical, gravity axis).
   *
   * @par Vocabulary
   * - **Stroke**: one rapid physical movement in one direction.
   * - **Peak**: an extreme of the on-axis acceleration. For a smooth shake, peaks occur at
   *   the turnarounds between strokes (where the movement reverses) and zero crossings
   *   occur mid-stroke, at maximum speed.
   * - **Peak amplitude**: the size of one peak (positive peak > 0, negative peak < 0).
   * - **Peak-to-peak amplitude**: positive peak amplitude minus negative peak amplitude.
   * - **Half-period**: the time between a positive and the following negative peak.
   * - **Sphere frame**: the local coordinate basis of the Qbead device.
   *   Local meaning it stays fixed w.r.t the device, thus transforming under user rotations
   *   in the global coordinate basis; e.g. the Z-axis stays aligned with the LED-poles.
   * - **World frame**: the global coordinate basis of the Qbead device.
   *   Global meaning it stays fixed w.r.t. the user's world, thus staying fixed under
   *   user rotations; e.g. the Z-axis stays aligned with the gravity axis.
   *   
   * @par Units and frames
   * All accelerations and amplitudes are in g. All final axis are unit vecors.
   * All vectors passed to and stored in this struct are expressed in the sphere frame.
   *
   * @par Method
   * Each sample has the gravity estimate subtracted, and the remainder is split into
   * an on-axis part (along shakeAxis) and an off-axis part (perpendicular to it).
   * A peak is registered when the on-axis part is large enough while the off-axis part
   * is small enough; i.e. the sampled shake is within a tolerance cone around the intended axis.
   * A shake is reported when a positive and a negative peak occur within 
   * maxHalfPeriodMs of each other and their peak-to-peak amplitude is large enough.
   *
   * @par Usage
   * Call update() exactly once per sensor sample, with the unsmoothed acceleration;
   * smoothing such as the 100 ms exp-LPF in readIMU removes the fast strokes this 
   * detector looks for. With externalGravity, gap handling is the gravity estimator's job;
   * the isInitialised flag here only matters for the internal gravity filter. 
   * Without externalGravity, set isInitialised to false to recallibrate the gravityEstimate.
   */
  struct ShakeDetector {
    /** @name Detection behaviour tunable parameters 
     *  @{ */

    /**
     * Minimum amplitude each peak must reach, per side. The total swing is therefore
     * at least 2 x this value. Raise it to demand harder strokes.
     */
    float minPeakAmplitude = 0.5f;

    /**
     * Minimum peak-to-peak amplitude (positive peak minus negative peak).
     * It only has an effect when it exceeds 2 x minPeakAmplitude; it then allows
     * asymmetric shakes (e.g. +0.5 and -1.4) while still demanding a large total swing.
     */
    float minPeakToPeakAmplitude = 1.9f;

    /**
     * On-axis acceleration below which a sample counts as close to the shake signal's zero crossing.
     * The last such sample before the first peak supplies shakeAxisAtStart.
     * Keep it above the noise and gravity-offset floor, and well below minPeakAmplitude.
     */
    float nearZeroThreshold = 0.1f; 

    /**
     * Maximum ratio of off-axis to on-axis acceleration for a peak to count.
     * It defines a cone around the shake axis with half-angle atan(ratio):
     * 0.45 is about 24 degrees. Smaller values demand a straighter shake.
     */
    float maxOffAxisRatio = 0.45f;

    /**
     * Maximum time between a positive and a negative peak (half a period of the shake).
     * 300 ms corresponds to a minimum shake frequency of roughly 1.7 Hz.
     */
    uint32_t maxHalfPeriodMs = 300;

    /** Minimum time between two reported shakes in ms. */
    uint32_t cooldownMs = 500;

    /**
     * How slowly the internal gravity estimate adapts in seconds; i.e. it is Tau, 
     * the time constant of a first-order low-pass filter: about 63% of a step change
     * is absorbed in 1 Tau.
     * Larger values follow rotations more slowly but are less disturbed by shaking.
     * Not used when externalGravity is passed to update().
     */
    float intGravityEstTimeConstantSeconds = 1.0f;
    /** @} */

    /** @name Saved outputs
     *  @{ */

    /**
     * Unit shake axis (sphere frame) captured at the last stable shake axis before a detected shake.
     * Useful for for acting upon the intended sphere basis before the shake action potentially displaced it.
     * For example we use it as the intended measurement axis in the case of shake to measure.
     * 
     * Important! Copy it immediately when update() returns true as subsequent update calls can overwrite it. 
     */
    float shakeAxisAtStart[3] = {0.f, 0.f, 1.f};
    /** @} */

    /** @name Internal state
     *  @{ */
    /** Gravity (sphere frame), subtracted from every sample. */
    float gravityEstimate[3] = {0.f, 0.f, 1.f}; 
    /** Unit axis the detector listens along; re-derived every update. */
    float shakeAxis[3] = {0.f, 0.f, 1.f};
    /** Continuously updating shakeAxis around nearZero before a shake is initiated. Becomes shakeAxisAtStart if not stale.*/
    float shakeAxisPrePrimaryPeak[3] = {0.f, 0.f, 1.f};
    /** True once gravityEstimate holds valid data. Set to false to restart (e.g. after a timing gap). */   
    bool isInitialised = false;                 
    /** Largest positive on-axis acceleration in the current window. */
    float positivePeakAmplitude = 0.f;          
    /** Most negative on-axis acceleration in the current window (signed, <= 0). */
    float negativePeakAmplitude = 0.f;          
    /** True while a positive peak is inside the pairing window. */
    bool hasPositivePeak = false;               
    /** True while a negative peak is inside the pairing window. */
    bool hasNegativePeak = false;               
    /** Time of the most recent qualifying positive sample. */
    uint32_t positivePeakTimeMs = 0;            
    /** Time of the most recent qualifying negative sample. */
    uint32_t negativePeakTimeMs = 0;            
    /** Time of the last reported shake (for the cooldown). */
    uint32_t lastShakeTimeMs = 0;       
    /** Time of the last sample before a potential shake was initiated. */        
    uint32_t lastNearZeroTimeMs = 0;
    /** @} */

    /**
     * @brief Processes one accelerometer sample.
     *
     * @param accelSphere      Acceleration in the sphere frame (3 elements), in g.
     *                         Unfiltered acceleration data works best as strokes rely on high freqs.
     * @param dtSeconds        Time since the previous sample, in seconds.
     * @param nowMillis        Current time in ms (e.g. millis()). 
     *                         Unsigned arithmetic keeps the time windows correct across wraparound.
     * @param[in] externalGravity  Optional gravity vector in the sphere frame (3 elements).
     *                             If nullptr, gravity is filtered internally from accelSphere.
     * @return True exactly once per detected shake.
     */
    bool update(const float accelSphere[3], float dtSeconds, uint32_t nowMillis,
                const float* externalGravity = nullptr)
    {
      // 1. Gravity reference: use the external estimate, or seed/filter our own.
      if (externalGravity) {
        memcpy(gravityEstimate, externalGravity, sizeof gravityEstimate);
        isInitialised = true;
      } else if (!isInitialised) {
        memcpy(gravityEstimate, accelSphere, sizeof gravityEstimate);
        isInitialised = true;
      }

      // 2. Shake axis. Currently this is the gravity direction (vertical shakes).
      //    FUTURE: replace this block to select another axis (e.g. world X or Y).
      float gravityNorm = sqrtf(gravityEstimate[0]*gravityEstimate[0] +
                                gravityEstimate[1]*gravityEstimate[1] +
                                gravityEstimate[2]*gravityEstimate[2]);
      // guard against a zero vector (ex. bad I2C read, free-fall..)
      if (gravityNorm < 1e-6f) {
        isInitialised = false; // re-seed on next sample
        return false; 
      }
      // update the gravity unit vector
      for (int i = 0; i < 3; i++) shakeAxis[i] = gravityEstimate[i] / gravityNorm;

      // 3. Remove gravity, then split into on-axis and off-axis parts.
      //    At rest the accelerometer reads 1 g "up"; subtracting the gravity estimate leaves ~0 (i.e. only the user-induced acceleration).
      float linearAccel[3] = {accelSphere[0] - gravityEstimate[0],
                              accelSphere[1] - gravityEstimate[1],
                              accelSphere[2] - gravityEstimate[2]};
      // projection onto detection axis; requires shakeAxis to be unit vector. Retains sign.
      float onAxisAccelAmp = linearAccel[0]*shakeAxis[0] + linearAccel[1]*shakeAxis[1] + linearAccel[2]*shakeAxis[2];
      // removing parallel part leaves us with the perpendicular component as a vector
      float offAxisVector[3] = {linearAccel[0] - onAxisAccelAmp*shakeAxis[0],
                                linearAccel[1] - onAxisAccelAmp*shakeAxis[1],
                                linearAccel[2] - onAxisAccelAmp*shakeAxis[2]};
      // We only need the off-axis length for the cone test so take the norm
      float offAxisAccelAmp = sqrtf(offAxisVector[0]*offAxisVector[0] +
                                    offAxisVector[1]*offAxisVector[1] +
                                    offAxisVector[2]*offAxisVector[2]);

      // 4. Internal gravity filter (only without externalGravity). 
      //    Less accurate than gravityTracker but doesn't require gyro (higher power consumption).
      if (!externalGravity) {
        float filterGain = dtSeconds / ((hasPositivePeak || hasNegativePeak)
                                        ? 4 * intGravityEstTimeConstantSeconds // slow down mid-shake to preserve gravity estimate
                                        : intGravityEstTimeConstantSeconds);
        for (int i = 0; i < 3; i++)
          gravityEstimate[i] += filterGain * (accelSphere[i] - gravityEstimate[i]);
      }

      // 5. Forget peaks older than the pairing window.
      if (hasPositivePeak && nowMillis - positivePeakTimeMs > maxHalfPeriodMs) hasPositivePeak = false;
      if (hasNegativePeak && nowMillis - negativePeakTimeMs > maxHalfPeriodMs) hasNegativePeak = false;

      // 6. Register a qualifying sample: large enough and inside the cone around the axis.
      bool awaitingFirstPeak = !(hasPositivePeak || hasNegativePeak);
      float onAxisAbsAmp = fabsf(onAxisAccelAmp);
      // continuously track stable shakeAxis estimates before any shake is initiated.
      if (awaitingFirstPeak && onAxisAbsAmp < nearZeroThreshold){
        memcpy(shakeAxisPrePrimaryPeak, shakeAxis, sizeof shakeAxis);
        lastNearZeroTimeMs = nowMillis;
      }
      // logic for during an initiated shake attempt.
      if (onAxisAbsAmp > minPeakAmplitude && offAxisAccelAmp < maxOffAxisRatio * onAxisAbsAmp) {
        if (awaitingFirstPeak) 
        {
          // safety check for staleness, if failed fall back on current live shakeAxis.
          // current maxHalfPeriodMs is generous and mostly targets repeated too slow attempts preceding a fast one.
          bool nearZeroIsRecent = nowMillis - lastNearZeroTimeMs <= maxHalfPeriodMs; 
          memcpy(shakeAxisAtStart, nearZeroIsRecent ? shakeAxisPrePrimaryPeak : shakeAxis, sizeof shakeAxis);
        }
        if (onAxisAccelAmp > 0) {
          if (!hasPositivePeak || onAxisAccelAmp > positivePeakAmplitude) positivePeakAmplitude = onAxisAccelAmp;
          positivePeakTimeMs = nowMillis;
          hasPositivePeak = true;
        } else {
          if (!hasNegativePeak || onAxisAccelAmp < negativePeakAmplitude) negativePeakAmplitude = onAxisAccelAmp;
          negativePeakTimeMs = nowMillis;
          hasNegativePeak = true;
        }
      }

      // 7. Report a shake: Both peaks present, large enough swing, cooldown over.
      float peakToPeakAmplitude = positivePeakAmplitude - negativePeakAmplitude;
      if (hasPositivePeak && hasNegativePeak &&
          peakToPeakAmplitude > minPeakToPeakAmplitude &&
          nowMillis - lastShakeTimeMs > cooldownMs) {
        hasPositivePeak = hasNegativePeak = false;
        lastShakeTimeMs = nowMillis;
        return true;
      }
      return false;
    }
  };


  /**
   * @brief Tracks the gravity direction by fusing gyroscope and accelerometer.
   *
   * Each step rotates the estimate with the gyro (accurate during motion) and then
   * pulls it slowly toward the accelerometer (corrects drift). The pull is weighted
   * by how close |accel| is to 1 g, so shakes and impacts barely disturb it.
   * All vectors are in the sphere frame; the axis mapping must be a proper rotation
   * (determinant +1), as for your ix/iy/iz and sx/sy/sz.
   */
  //struct GravityTracker {
  //  float g[3] = {0, 0, 1}; bool init = false;
  //  void update(const float a[3], const float w_dps[3], float dt) {   // chip frame!
  //    const float D2R = 0.0174533f, TAU = 1.5f;
  //    if (!init) { memcpy(g, a, sizeof g); init = true; return; }
  //    float w[3] = {w_dps[0]*D2R, w_dps[1]*D2R, w_dps[2]*D2R};
  //    float c[3] = {w[1]*g[2]-w[2]*g[1], w[2]*g[0]-w[0]*g[2], w[0]*g[1]-w[1]*g[0]};
  //    for (int i = 0; i < 3; i++) g[i] -= c[i]*dt;                    // predict
  //    float am  = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
  //    float k   = (dt/TAU) * fmaxf(0.f, 1.f - fabsf(am - 1.f)/0.3f);   // trust accel near 1 g
  //    for (int i = 0; i < 3; i++) g[i] += k*(a[i] - g[i]);            // correct
  //    float gm = sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
  //    for (int i = 0; i < 3; i++) g[i] /= gm;
  //  }
  //};

  /**
   * @brief Tracks the gravity direction by fusing gyroscope and accelerometer readings.
   * 
   * Because the accelerometer measures gravity and the device's motion simultaneously,
   * it can't seperate them risking pollution of gravity direction estimates by device motion.
   * Meanwhile the gyro measures instantaneous device rotation accurately, but integrated over time
   * it starts to drift. Hence, for a more accurate estimate we employ both at their strengths while
   * covering their weaknesses with each other.
   *
   * @par Output
   * gravityEstimate is the world frame "up" direction expressed in sphere frame coordinates, 
   * as a unit vector. It points opposite to the gravitational acceleration, because that is what
   * a resting accelerometer measures. As the sphere rotates, its coordinates change. 
   *
   * @par Method
   * Each step has two stages. **Predict**: rotate the estimate with the gyro (accurate during
   * motion). **Correct**: pull it slowly toward the accelerometer (removes drift). The pull is
   * weighted by how close |accel| is to 1 g, so shakes and impacts barely disturb it and thus
   * during such times the estimate coasts on the gyro. After returning to rest the built up
   * gyro drift is cancelled out over a few seconds.  
   *
   * @par Units and frames
   * Accelerations in g, angular rates in degrees/s, times in seconds. All vectors are in the
   * sphere frame. The chip-to-sphere axis mapping must be a proper rotation (determinant +1),
   * as it is for the current ix/iy/iz and sx/sy/sz.
   *
   * @par Usage
   * Call update() once per sensor sample with the unsmoothed acceleration and the
   * bias-corrected gyro rate. Only use gravityEstimate when isInitialised is true.
   * A long gap in sampling makes the estimate invalid until a trustworthy sample re-seeds it.
   */
  struct GravityTracker {
    /** @name Tracking behaviour tunable parameters
     *  @{ */

    /** 
     * The time constant (Tau) of a first-order low-pass filter: about 63% of a step change
     * is absorbed in 1 Tau.
     * How slowly to introduce accelerometer correction. Larger trusts the gyro for longer. 
     */
    float accelCorrectionTimeConstantSeconds = 1.5f;

    /**
     * Accelerometer trust band, in g. The correction weight falls linearly to 0 as |accel|
     * deviates from 1 g by this amount, so shakes and impacts are ignored.
     */
    float accelTrustBand = 0.3f;

    /**
     * Longest time step (s) that is integrated. A longer gap invalidates the estimate
     * (isInitialised becomes false) instead of integrating a stale gyro rate.
     */
    float maxGapSeconds = 0.1f;

    /** The estimate is (re)seeded only from a sample whose |accel| is within this many g of 1 g. */
    float seedTrustBand = 0.1f;
    /** @} */

    /** @name State
     *  @{ */
    /** Unit "up" direction in the sphere frame. Valid only when isInitialised. */
    float gravityEstimate[3] = {0.f, 0.f, 1.f};
    /** False at start and after a gap, until a trustworthy sample re-seeds the estimate. */
    bool isInitialised = false;                 
    /** @} */

    /**
     * @brief Advances the estimate by one sample.
     *
     * @param accelSphere   Acceleration in the sphere frame (3 elements), in g. Unsmoothed.
     * @param gyroSphereDps Bias-corrected angular rate in the sphere frame (3 elements), in degrees/s.
     * @param dtSeconds     Time since the previous sample, in seconds.
     */
    void update(const float accelSphere[3], const float gyroSphereDps[3], float dtSeconds)
    {
      float accelMagnitude = sqrtf(accelSphere[0]*accelSphere[0] +
                                   accelSphere[1]*accelSphere[1] +
                                   accelSphere[2]*accelSphere[2]);
      float accelMagnitudeError = fabsf(accelMagnitude - 1.f);    // deviation from 1 g

      // 1. A gap in sampling makes the integrated history unreliable.
      if (dtSeconds > maxGapSeconds) isInitialised = false;

      // 2. (Re)seed from the accelerometer, but only when it is trustworthy.
      if (!isInitialised) {
        if (accelMagnitudeError < seedTrustBand) {
          for (int i = 0; i < 3; i++) gravityEstimate[i] = accelSphere[i] / accelMagnitude;
          isInitialised = true;
        }
        return;  // otherwise keep the stale estimate and wait
      }

      // 3. Predict: in the sphere frame a world-fixed vector rotates as -omega x gravity.
      float omegaRadPerSecond[3] = {gyroSphereDps[0]*DEG_TO_RAD,
                                    gyroSphereDps[1]*DEG_TO_RAD,
                                    gyroSphereDps[2]*DEG_TO_RAD};
      float crossProduct[3] = {
        omegaRadPerSecond[1]*gravityEstimate[2] - omegaRadPerSecond[2]*gravityEstimate[1],
        omegaRadPerSecond[2]*gravityEstimate[0] - omegaRadPerSecond[0]*gravityEstimate[2],
        omegaRadPerSecond[0]*gravityEstimate[1] - omegaRadPerSecond[1]*gravityEstimate[0]};
      for (int i = 0; i < 3; i++) gravityEstimate[i] -= crossProduct[i] * dtSeconds;

      // 4. Correct: pull toward the accelerometer, weighted by how much it can be trusted.
      float accelTrust = fmaxf(0.f, 1.f - accelMagnitudeError / accelTrustBand);   // 1 at 1 g, 0 beyond the band
      float correctionGain = (dtSeconds / accelCorrectionTimeConstantSeconds) * accelTrust;
      for (int i = 0; i < 3; i++)
        gravityEstimate[i] += correctionGain * (accelSphere[i] - gravityEstimate[i]);

      // 5. Renormalise to unit length (guarded against a zero vector).
      float norm = sqrtf(gravityEstimate[0]*gravityEstimate[0] +
                         gravityEstimate[1]*gravityEstimate[1] +
                         gravityEstimate[2]*gravityEstimate[2]);
      if (norm > 1e-6f)
        for (int i = 0; i < 3; i++) gravityEstimate[i] /= norm;
    }
  };

  class Qbead
  {
  public:
    Qbead(const uint16_t pin00 = QB_LEDPIN,
          const uint16_t pixelconfig = QB_PIXELCONFIG,
          const uint16_t nsections = QB_NSECTIONS,
          const uint16_t nlegs = QB_NLEGS,
          const uint8_t imu_addr = QB_IMU_ADDR,
          const uint8_t ix = QB_IX,
          const uint8_t iy = QB_IY,
          const uint8_t iz = QB_IZ,
          const bool sx = QB_SX,
          const bool sy = QB_SY,
          const bool sz = QB_SZ)
        : imu(LSM6DS3(I2C_MODE, imu_addr)),
          pixels(Adafruit_NeoPixel(nlegs * (nsections - 1) + 2, pin00, pixelconfig)),
          nsections(nsections),
          nlegs(nlegs),
          theta_quant(180 / nsections),
          phi_quant(360 / nlegs),
          ix(ix), iy(iy), iz(iz),
          sx(sx), sy(sy), sz(sz),
          shake(),
          gravity()
    {
    }

    static Qbead *callbackTarget; // we need a global singleton static instance because bluefruit callbacks do not support context variables -- thankfully this is fine because there is indeed only one Qbead in existence at any time

    LSM6DS3 imu;
    ShakeDetector shake;
    GravityTracker gravity;
    Adafruit_NeoPixel pixels;
    BLEManager::BLEManager ble;
    BlochVector innerStates[INNER_STATE_COUNT];
    uint8_t innerStateCount = 0;

    const uint8_t nsections;
    const uint8_t nlegs;
    const uint8_t theta_quant;
    const uint8_t phi_quant;
    const uint8_t ix, iy, iz;
    const bool sx, sy, sz;
    float rbuffer[3];
    float whentapped_buffer[3];
    float x_whentapped, y_whentapped, z_whentapped; // set when wasTapped is called
    float x, y, z, rx, ry, rz;                      // filtered and raw acc, in units of g
    float t_acc, p_acc;                             // theta and phi according to gravity
    // TODO
    float gyroBias[3] = {0,0,0}; // auto learns zero-rate level of gyro to subtract for accuracy
    float whenshaken_buffer[3]; // gravity at the moment of the shake
    float x_whenshaken, y_whenshaken, z_whenshaken;
    //float T_imu;                                    // last update from the IMU TODO
    uint32_t T_imu;
    volatile bool tapped = false;
    volatile bool tappedrecorded = false;
    bool shaken = false;

    // TODO (added volatile)
    uint32_t stateColours[INNER_STATE_COUNT] = {
        color(0, 0, 255),   // Blue
        color(255, 0, 0),   // Red
        color(0, 255, 0),   // Green
        color(255, 255, 0), // Yellow
        color(255, 0, 255), // Magenta
        color(255, 128, 0)  // Orange
    };
    uint32_t cyclingIndex = 0;
    uint32_t lastChange = 0;


    void setupIMUTapDetection()
    {
      // TODO already done in begin() using settings struct
      // Turn on the accelerometer
      // Acc = 416Hz (High-Performance mode)
      //imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, LSM6DS3_ACC_GYRO_ODR_XL_416Hz);
      // TODO

      // Optionally, disable gyroscope to save power
      // imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN);

      // Enable tap detection in X,Y,Z:
      uint8_t TAP_CFG_SETTING = LSM6DS3_ACC_GYRO_TIMER_EN_ENABLED | LSM6DS3_ACC_GYRO_TAP_Z_EN_ENABLED | LSM6DS3_ACC_GYRO_TAP_Y_EN_ENABLED | LSM6DS3_ACC_GYRO_TAP_X_EN_ENABLED;
      imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, TAP_CFG_SETTING);

      // Set shock time window:
      imu.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, LSM6DS3_ACC_GYRO_SHOCK_MASK & 0b11);

      // Set tap threshold:
      // TODO
      //uint8_t thrshold_setting = 8; // number between 0 and 31
      uint8_t threshold_setting = 2;
      // TODO
      imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, threshold_setting);

      // Only do single tap detection. Seems like the naming is incorrect?
      // TODO: INDEED INCORRECT, Library wrapper implemented reverse from spec sheet
      imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP);

      // Single-tap interrupt driven to pin 1
      imu.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_ENABLED);

      // TODO
      // Enable low pass filter and set cutoff frequency to datarate/400
      imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, LSM6DS3_ACC_GYRO_LPF2_XL_EN | LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_100);
      //imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x60);
      //imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x80);
      //imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x00);
      // TODO

      // Setup interrupt callback
      pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
      attachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1), tap_isr, RISING);

      Serial.println("Enabled IMU interrupt!");
    }

    void begin()
    {
      callbackTarget = this;
      Serial.begin(9600);
      //while (!Serial)
      //  ; // TODO some form of warning or a way to give up if Serial never becomes available
      unsigned long t0 = millis();
      while (!Serial && millis() - t0 < 15000) { ; }

      pixels.begin();
      clear();
      setBrightness(10);

      // TODO
      // These settings are used by the imu class. Only directly setting the registers as in
      // setup_IMU_Tap_detection doesn't also schange the settings struct.
      // While IMU.begin does copy the settings struct
      imu.settings.accelRange      = 8;     // ±8 g: no clipping on shakes
      imu.settings.accelSampleRate = 416;
      imu.settings.accelBandWidth  = 400;   // keeps your current 400 Hz analog BW
      imu.settings.gyroEnabled = true; // maybe off?
      // TODO

      Serial.println("[INFO] Booting... Qbead on XIAO BLE Sense + LSM6DS3 compiled on " __DATE__ " at " __TIME__);
      if (!imu.begin())
      {
        Serial.println("[DEBUG]{IMU} IMU initialized correctly");
      }
      else
      {
        Serial.println("[ERROR]{IMU} IMU failed to initialize");
      }

      setupIMUTapDetection();

      // TODO
      uint8_t id; imu.readRegister(&id, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
      Serial.println(id, HEX);
      // TODO

      ble.beginDualRole();
    }

    void clear()
    {
      pixels.clear();
    }

    void show()
    {
      pixels.show();
    }

    void setLegPixelColor(int leg, int pixel, uint32_t color)
    {
      int mappedPixelIndex = computePixelIndex(leg, pixel);
      pixels.setPixelColor(mappedPixelIndex, color);
    }

    uint32_t getLegPixelColor(int leg, int pixel)
    {
      int mappedPixelIndex = computePixelIndex(leg, pixel);
      return pixels.getPixelColor(mappedPixelIndex);
    }

    void addLegPixelColor(int leg, int pixel, uint32_t c0)
    {
      uint32_t c1 = getLegPixelColor(leg, pixel);
      setLegPixelColor(leg, pixel, addColor(c0, c1));
    }

    void setBrightness(uint8_t b)
    {
      pixels.setBrightness(b);
    }

    void setBloch_deg(const BlochVector &state, uint32_t color)
    {
      setBloch_deg(state.theta, state.phi, color);
    }

    void setBloch_deg(float theta, float phi, uint32_t color)
    {
      float theta_section = theta / theta_quant;
      if (theta_section < 0.5)
      {
        setLegPixelColor(0, 0, color);
      }
      else if (theta_section > nsections - 0.5)
      {
        setLegPixelColor(0, nsections, color);
      }
      else
      {
        int theta_int = min(nsections - 1, round(theta_section)); // to avoid precision issues near the end of the range
        int phi_int = round(phi / phi_quant);
        phi_int = phi_int > nlegs - 1 ? 0 : phi_int;
        setLegPixelColor(phi_int, theta_int, color);
      }
    }

    /*
    // TODO needs brightness correction for when we have many LEDs on (when they are denser)
    // TODO make sure you are not in situations where no LEDs are lit because none are close (an edge case of brightness correction above)
    // TODO skip far-away pixels so the loop is not so expensive
    void setBloch_deg_smooth(const BlochVector& state, uint32_t color) {
      float width = 40;

      for (int phi_pixel = 0; phi_pixel <= nlegs; ++phi_pixel) {
        for (int theta_pixel = 0; theta_pixel <= nsections; ++theta_pixel) {
          float internal_angle = state.centralAngle(BlochVector(theta_quant * theta_pixel, phi_quant * phi_pixel));
          if (internal_angle > width) {
            continue;
          }

          float brightness = 1 - internal_angle * internal_angle / (width * width);

          addLegPixelColor(phi_pixel, theta_pixel, scaleColor(brightness, color));
        }
      }
    }

    void setBloch_deg_smooth(float theta, float phi, uint32_t color) {
      setBloch_deg_smooth(BlochVector(theta, phi), color);
    }
    */

    void setBloch_deg_smooth(const BlochVector &state, uint32_t color)
    {
      setBloch_deg_smooth(state.theta, state.phi, color);
    }

    void setBloch_deg_smooth(float theta, float phi, uint32_t c)
    {
      if (!checkThetaAndPhi(theta, phi))
        return;
      float theta_section = theta / theta_quant;
      int theta_int = min(nsections - 1, round(theta_section)); // to avoid precision issues near the end of the range
      int phi_int = round(phi / phi_quant);
      phi_int = phi_int > nlegs - 1 ? 0 : phi_int;

      float p = (theta_section - theta_int);
      int theta_direction = sign(p);
      p = abs(p);
      float q = 1 - p;
      p = p * p;
      q = q * q;

      uint8_t rc = redch(c);
      uint8_t gc = greench(c);
      uint8_t bc = bluech(c);

      setLegPixelColor(phi_int, theta_int, color(q * rc, q * gc, q * bc));
      setLegPixelColor(phi_int, theta_int + theta_direction, color(p * rc, p * gc, p * bc));
    }

    void testPixels()
    {
      Serial.println("[INFO] Testing all pixels discretely");
      for (int i = 0; i < pixels.numPixels(); i++)
      {
        pixels.setPixelColor(i, color(255, 255, 255));
        pixels.show();
        delay(5);
      }
      Serial.println("[INFO] Testing smooth transition between pixels");
      for (int phi = 0; phi < 360; phi += 30)
      {
        for (int theta = 0; theta < 180; theta += 6)
        {
          clear();
          setBloch_deg_smooth(theta, phi, colorWheel_deg(phi));
          show();
        }
      }
    }

    bool wasTapped()
    {
      if (!tappedrecorded)
        return false;
      // return true if IMU detected a tap since the last time wasTapped() was called
      bool wasTapped = tapped;
      // set tapped to false, so that the next time this function is called, it will return false
      tapped = false;
      tappedrecorded = false;
      // save tapped location
      if (wasTapped)
      {
        // TODO: ALready in Sphere coordinates
        //x_whentapped = whentapped_buffer[ix];
        //y_whentapped = whentapped_buffer[iy];
        //z_whentapped = whentapped_buffer[iz];
        x_whentapped = whentapped_buffer[0];
        y_whentapped = whentapped_buffer[1];
        z_whentapped = whentapped_buffer[2];
        // TODO
      }
      return wasTapped;
    }

    bool wasShaken()
    {
      if (!shaken) return false;
      shaken = false;
      x_whenshaken = whenshaken_buffer[0];
      y_whenshaken = whenshaken_buffer[1];
      z_whenshaken = whenshaken_buffer[2];
      return true;
    }

    BLEManager::DataPacket takeLatestPacket()
    {
      BLEManager::DataPacket packet;
      if (!ble.takePacket(packet))
      {
        return {BLEManager::CommandType::None, 0};
      }
      return packet;
    }

    static void tap_isr()
    {
      // This function is called when the IMU triggers an interrupt. That is: when a tap is detected!
      // We have to refer to the callbackTarget of the qbead here,
      // because the one and only qbead object does not exist when this isr is defined.
      // Then readout XYZ immediately
      // We could choose to only readout XYZ when we haven't yet processed the last tap,
      // but for now, let's just update the position everytime we tap.
      callbackTarget->tappedrecorded = false;
      callbackTarget->tapped = true;
    }



    void readIMU(bool print = true)
    {
      rbuffer[0] = imu.readFloatAccelX();
      rbuffer[1] = imu.readFloatAccelY();
      rbuffer[2] = imu.readFloatAccelZ();
      rx = (1 - 2 * sx) * rbuffer[ix];
      ry = (1 - 2 * sy) * rbuffer[iy];
      rz = (1 - 2 * sz) * rbuffer[iz];
      float rawmag2 = rx * rx + ry * ry + rz * rz;

      uint32_t T_new = micros(); //TODO: Can lose resolution after ~3-4h
      uint32_t delta = T_new - T_imu;
      float dt = delta*1e-6f;
      T_imu = T_new;
      const float T = 100000; // 100 ms // TODO make the filter timeconstant configurable

      // TODO
      // Gravity tracking
      // Gravity tracking + shake detection (exactly one shake.update per call)
      float wc[3] = { imu.readFloatGyroX(), imu.readFloatGyroY(), imu.readFloatGyroZ() };  // dps, chip frame
      float am = sqrtf(rawmag2);
      float wm2 = 0;
      for (int i = 0; i < 3; i++) { float d = wc[i] - gyroBias[i]; wm2 += d*d; }
      if (fabsf(am - 1.f) < 0.03f && wm2 < 9.f)                       // still: learn bias
        for (int i = 0; i < 3; i++) gyroBias[i] += 0.002f * (wc[i] - gyroBias[i]);
      float w[3] = { (1-2*sx)*(wc[ix]-gyroBias[ix]),
                     (1-2*sy)*(wc[iy]-gyroBias[iy]),
                     (1-2*sz)*(wc[iz]-gyroBias[iz]) };
      float r[3] = {rx, ry, rz};

      if (delta > T)
      {
        gravity.init = false;     // restart both estimators after a gap
        shake.init = false;
      }
      else
      {
        gravity.update(r, w, dt);
        if (shake.update(r, dt, millis(), gravity.g))
        {
          //whenshaken_buffer[0] = gravity.g[0];   // or shake.trigAx_atShakeStart[...] if you added the start-of-shake snapshot
          //whenshaken_buffer[1] = gravity.g[1];
          //whenshaken_buffer[2] = gravity.g[2];
          whenshaken_buffer[0] = shake.trigAx_atShakeStart[0];
          whenshaken_buffer[1] = shake.trigAx_atShakeStart[1];
          whenshaken_buffer[2] = shake.trigAx_atShakeStart[2];
          shaken = true;
        }
      }
      // TODO


      if (delta > T)
      {
        x = rx;
        y = ry;
        z = rz;
      }
      else
      {
        float d = delta / T;
        x = d * rx + (1 - d) * x;
        y = d * ry + (1 - d) * y;
        z = d * rz + (1 - d) * z;
      }
      float mag2 = x * x + y * y + z * z;

      // TODO
      // polar and azimuth angles of the smoothed acceleration vector in the sensor frame
      // NOT rotational accellerations values. Maybe rename?
      // t_acc = theta(x, y, z) * 180 / 3.14159;
      // p_acc = phi(x, y) * 180 / 3.14159;
      t_acc = theta(x, y, z) * RAD_TO_DEG;
      p_acc = phi(x, y) * RAD_TO_DEG;
      // TODO
      if (p_acc < 0)
      {
        p_acc += 360;
      } // to bring it to [0,360] range

      if (!tappedrecorded && tapped)
      {
        tappedrecorded = true;
        // TODO: Why re-read this info without it being ex-LPF'ed???
        //whentapped_buffer[0] = imu.readFloatAccelX();
        //whentapped_buffer[1] = imu.readFloatAccelY();
        //whentapped_buffer[2] = imu.readFloatAccelZ();

        whentapped_buffer[0] = x;
        whentapped_buffer[1] = y;
        whentapped_buffer[2] = z;
      }

      if (print)
      {
        Serial.print(tappedrecorded);
        Serial.print("\t");
        Serial.print(tapped);
        Serial.print("\t");
        Serial.print(x);
        Serial.print("\t");
        Serial.print(y);
        Serial.print("\t");
        Serial.print(z);
        Serial.print("\t");
        Serial.print(mag2);
        Serial.print("\t");
        Serial.print(rawmag2);
        Serial.print("\t-1\t1\t");
        Serial.print(t_acc);
        Serial.print("\t");
        Serial.print(p_acc);
        Serial.print("\t0\t360\t");
        Serial.println();
      }

      rbuffer[0] = x;
      rbuffer[1] = y;
      rbuffer[2] = z;
    }

    bool hasState(const BlochVector &state,
                  float thetaTolerance = 1.0f,
                  float phiTolerance = 1.0f) const
    {
      for (uint8_t i = 0; i < innerStateCount; i++)
      {
        const BlochVector &item = innerStates[i];

        float thetaDifference = fabsf(item.theta - state.theta);

        float phiDifference = fabsf(item.phi - state.phi);
        phiDifference = min(phiDifference, 360.0f - phiDifference);

        if (thetaDifference <= thetaTolerance &&
            phiDifference <= phiTolerance)
        {
          return true;
        }
      }

      return false;
    }

    void addState(BlochVector &state)
    {
      BlochVector newState(state.theta, state.phi);
      if (!hasState(newState) && innerStateCount < INNER_STATE_COUNT)
      {
        innerStates[innerStateCount++] = newState;
      }
    }

    void clearStates()
    {
      innerStateCount = 0;
    }

    void applyPreparedState(uint32_t state)
    {
      BlochVector up(0, 0);
      BlochVector down(180, 0);
      if (state == 1)
      {
        this->clearStates();
        this->addState(up);
        this->addState(down);
      }
      else if (state == 2)
      {
        this->clearStates();
        this->addState(down);
        this->addState(up);
      }
    }

    void displayCurrentStatesStatic()
    {
      this->clear();
      for (uint8_t i = 0; i < innerStateCount; i++)
      {
        const BlochVector &item = innerStates[i];
        const uint32_t &itemColour = stateColours[i];

        this->setBloch_deg(item, itemColour);
      }
      this->show();
    }

    void displayCurrentStatesCycling()
    {
      if (innerStateCount == 0)
      {
        return;
      }
      uint32_t currentTime = millis();
      uint32_t deltaTime = currentTime - lastChange;
      if (deltaTime < CYCLING_TIME)
      {
        return;
      }
      this->clear();
      lastChange = currentTime;
      this->setBloch_deg(innerStates[cyclingIndex], stateColours[cyclingIndex]);
      this->show();
      cyclingIndex = (cyclingIndex + 1) % (innerStateCount);
    }
  }; // end class

  Qbead *Qbead::callbackTarget = nullptr;

} // end namespace

#endif // QBEAD_H