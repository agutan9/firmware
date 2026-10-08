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
    * Number of alternating-sign peaks required before a shake is reported.
    * One stroke gives 2 (accelerate, brake), down-and-up gives 3, down-up-down gives 4.
    */
    uint8_t minAlternatingPeaks = 3;

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
    /** Sign of the most recent peak (+1 or -1); 0 if none yet. */
    int8_t lastPeakSign = 0;
    /** Number of sign changes in the current attempt, counting the first peak. */
    uint8_t alternatingPeakCount = 0;
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
      //    TODO PR: This can be extended to track world X or Y. 
      //    However, these are not as constant as world Z and would need some further work
      //    1. If you rotate the way you are interfacing with the device the world X or Y
      //       would not notice. If you turn 90deg your left-right (X) is now mapped to
      //       what was front-back (Y)
      //    2. Hence we would likely need a callibration step. Not impossible to implement,
      //       but also not trivial
      //    3. Any-shake in the X-Y plane would get around this but also requires some
      //       non-trivial extension to current code
      //    4. Without a magnometer the axis might drift over-time (would need further looking into)
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
        // never extrapolate past the measurement after a long gap
        filterGain = fminf(filterGain, 1.0f);   
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
      if (onAxisAbsAmp > minPeakAmplitude && offAxisAccelAmp < maxOffAxisRatio * onAxisAbsAmp) 
      {
        int8_t peakSign = (onAxisAccelAmp > 0) ? 1 : -1;
        // Stroke ramping up towards first valid peak 
        if (awaitingFirstPeak) 
        {
          // safety check for staleness, if failed fall back on current live shakeAxis.
          // current maxHalfPeriodMs is generous and mostly targets repeated too slow attempts preceding a fast one.
          bool nearZeroIsRecent = nowMillis - lastNearZeroTimeMs <= maxHalfPeriodMs; 
          memcpy(shakeAxisAtStart, nearZeroIsRecent ? shakeAxisPrePrimaryPeak : shakeAxis, sizeof shakeAxis);
          // init/reset peak counting
          alternatingPeakCount = 0;
          lastPeakSign = 0;
        }
        if (peakSign != lastPeakSign) 
        { 
          // Don't double count the same (increasing) peak
          alternatingPeakCount++; 
          lastPeakSign = peakSign; 
        }
        // Post first peak we detect until desired peak count is reached
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
          alternatingPeakCount >= minAlternatingPeaks &&
          peakToPeakAmplitude > minPeakToPeakAmplitude &&
          nowMillis - lastShakeTimeMs > cooldownMs) 
      {
        hasPositivePeak = hasNegativePeak = false;
        alternatingPeakCount = 0; 
        lastPeakSign = 0;
        lastShakeTimeMs = nowMillis;
        return true;
      }
      return false;
    }
  };

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
          sx(sx), sy(sy), sz(sz)
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
    float whentapped_buffer[3]= {0.f, 0.f, 1.f};
    float whenshaken_buffer[3]= {0.f, 0.f, 1.f};    /**< Unit shake axis (sphere frame) at the start of the last shake. */
    float gyroBiasDps[3] = {0.f, 0.f, 0.f};         /**< Learned gyro zero-rate offset (deg/s), subtracted from every reading. */
    float x_whentapped, y_whentapped, z_whentapped; /**< set when wasTapped is called */
    float x_whenshaken, y_whenshaken, z_whenshaken; /**< set when wasShaken is called */
    float x, y, z, rx, ry, rz;                      // filtered and raw acc, in units of g
    float t_acc, p_acc;             // TODO: why put these here? is this comment even correct?? ->     // theta and phi according to gravity
    volatile bool tapped = false;
    volatile bool tappedrecorded = false;
    bool shaken = false;
    uint32_t T_imu; // last update from the IMU


    bool localEntangleRequestPending = false;
    uint32_t localEntangleRequestStartedAtMs = 0;
    bool remoteEntangleRequestPending = false;
    uint32_t remoteEntangleRequestReceivedAtMs = 0;

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

  
    // FUTURE: PR TODO
    // In order to actually be able to set gyroOn to false readIMU needs refactoring
    // to not use gravityTracker and let shakeDetector use its internal acc only method
    /** Driver settings, written into the registers by imu.begin(). */
    void preConfigIMU_Settings(bool gyroOn = true)
    {
      imu.settings.accelRange      = 8;     // g: no clipping on shakes; tap threshold units scale with this
      imu.settings.accelSampleRate = 416;   // Hz, high-performance mode
      imu.settings.accelBandWidth  = 400;   // TR-C: LPF1 at ODR/2 (analog bandwidth is fixed at this ODR)
      imu.settings.gyroEnabled     = gyroOn;  // required by the gravity tracker
      imu.settings.gyroSampleRate  = 416;
      // imu.settings.gyroRange    = 1000;  // optional: finer resolution than the 2000 dps default
    }

    // PR TODO:
    // FUTURE: add switch case for different ODR_X bandwidhts?
    // Seems silly but is general to both tap and shake and there are many more filter registers which we currently don't set
    void postConfigIMU_Filters()
    {
      // Enable low pass filter and set cutoff frequency to datarate/100. Smaller cut-offs might attenuate shake detection.
      imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, LSM6DS3_ACC_GYRO_LPF2_XL_EN | LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_100);
    }

    // Sets the main isr interrupt
    void postConfigIMU_TapDetection()
    {
      // Enable tap detection in X,Y,Z:
      uint8_t TAP_CFG_SETTING = LSM6DS3_ACC_GYRO_TIMER_EN_ENABLED | LSM6DS3_ACC_GYRO_TAP_Z_EN_ENABLED | LSM6DS3_ACC_GYRO_TAP_Y_EN_ENABLED | LSM6DS3_ACC_GYRO_TAP_X_EN_ENABLED;
      imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, TAP_CFG_SETTING);

      // Set shock time window:
      imu.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, LSM6DS3_ACC_GYRO_SHOCK_MASK & 0b11);

      // Set tap threshold:
      uint8_t threshold_setting = 2; // number between 0 and 31
      imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, threshold_setting);

      // Only do single tap detection. Seems like the naming is incorrect?
      // TODO: INDEED INCORRECT, Library wrapper implemented reverse from spec sheet
      imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP);

      // Single-tap interrupt driven to pin 1
      imu.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_ENABLED);

      // Setup interrupt callback
      pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
      attachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1), tap_isr, RISING);

      Serial.println("Enabled IMU tap interrupt!");
    }

    void begin()
    {
      callbackTarget = this;
      Serial.begin(9600);
      // Wait for Serial to become available or hit a time-out before continuing
      unsigned long t0 = millis();
      while (!Serial && millis() - t0 < 15000)
      {
        ;
      }

      pixels.begin();
      clear();
      setBrightness(10);

      preConfigIMU_Settings(); // fills imu.settings; must run before imu.begin()

      Serial.println("[INFO] Booting... Qbead on XIAO BLE Sense + LSM6DS3 compiled on " __DATE__ " at " __TIME__);
      if (!imu.begin())
      {
        Serial.println("[DEBUG]{IMU} IMU initialized correctly");
      }
      else
      {
        Serial.println("[ERROR]{IMU} IMU failed to initialize");
      }

      postConfigIMU_Filters();
      postConfigIMU_TapDetection();
      // postConfigIMU_ShakeDetection: 
      // Shake is software based. Only relevant registers are the LPF due to high-freq acc attenuation

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
        // buffer already holds sphere-frame values; don't need ix/iy/iz
        x_whentapped = whentapped_buffer[0];
        y_whentapped = whentapped_buffer[1];
        z_whentapped = whentapped_buffer[2];
      }
      return wasTapped;
    }

    // localTrigger indicates the gesture for firing a local entanglement attempt
    bool entangle(bool localTrigger, uint32_t state)
    {
      const uint32_t now = millis();

      // Capture a newly received remote request.
      uint32_t receivedAtMs;
      if (ble.takeEntangleRequest(receivedAtMs))
      {
        remoteEntangleRequestPending = true;
        remoteEntangleRequestReceivedAtMs = receivedAtMs;

        Serial.println("[INFO]{ENTANGLE} Remote request queued");
      }

      // Capture a newly detected local tap and send our request.
      if (localTrigger)
      {
        localEntangleRequestPending = true;
        localEntangleRequestStartedAtMs = now;

        ble.sendData(BLEManager::CommandType::Entangle, 0, 0, 0);

        Serial.println("[INFO]{ENTANGLE} Local tap; request sent");
      }

      // Drop expired requests.
      if (localEntangleRequestPending &&
          (uint32_t)(now - localEntangleRequestStartedAtMs) > ENTANGLE_WINDOW_MS)
      {
        localEntangleRequestPending = false;
        Serial.println("[INFO]{ENTANGLE} Local request timed out");
      }

      if (remoteEntangleRequestPending &&
          (uint32_t)(now - remoteEntangleRequestReceivedAtMs) > ENTANGLE_WINDOW_MS)
      {
        remoteEntangleRequestPending = false;
        Serial.println("[INFO]{ENTANGLE} Remote request timed out");
      }

      // Entangle only if both requests are presently valid.
      if (localEntangleRequestPending && remoteEntangleRequestPending)
      {
        localEntangleRequestPending = false;
        remoteEntangleRequestPending = false;

        Serial.println("[INFO]{ENTANGLE} Requests matched");
        return applyPreparedState(state);
      }

      return false;
    }

    BLEManager::DataPacket takeLatestPacket()
    {
      BLEManager::DataPacket packet;
      if (!ble.takePacket(packet))
      {
        return {BLEManager::CommandType::None, 0, 0, 0};
      }
      return packet;
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

      uint32_t T_new = micros();
      uint32_t delta = T_new - T_imu;
      float dt = delta*1e-6f;
      T_imu = T_new;
      const float T = 100000; // 100 ms // TODO make the filter timeconstant configurable
      // PR: This is an old TODO olready present. Will refactor it before final PR merge

      // Process Gyro: read in the chip frame, learn the zero-rate offset while still, then map to the sphere frame.
      const float stillAccelTolerance = 0.03f;       // |accel| within this many g of 1 g counts as still
      const float stillGyroMaxSquaredDps2 = 9.f;     // (3 deg/s)^2: rotation below this counts as still
      const float biasLearningGain = 0.002f;         // fraction of the error absorbed per call

      float gyroChipDps[3] = { imu.readFloatGyroX(), imu.readFloatGyroY(), imu.readFloatGyroZ() };  // deg/s, chip frame
      float accelMagnitude = sqrtf(rawmag2);          // unit of g
      float gyroResidualSquaredDps2 = 0;              // squared rate left after bias removal
      for (int i = 0; i < 3; i++)
      {
        float gyroResidualDps = gyroChipDps[i] - gyroBiasDps[i];
        gyroResidualSquaredDps2 += gyroResidualDps * gyroResidualDps;
      }
      bool isStill = fabsf(accelMagnitude - 1.f) < stillAccelTolerance &&
                     gyroResidualSquaredDps2 < stillGyroMaxSquaredDps2;
      if (isStill)                                    // learn the zero-rate offset
        for (int i = 0; i < 3; i++)
          gyroBiasDps[i] += biasLearningGain * (gyroChipDps[i] - gyroBiasDps[i]);

      float gyroSphereDps[3] = { (1 - 2*sx) * (gyroChipDps[ix] - gyroBiasDps[ix]),
                                 (1 - 2*sy) * (gyroChipDps[iy] - gyroBiasDps[iy]),
                                 (1 - 2*sz) * (gyroChipDps[iz] - gyroBiasDps[iz]) };
      float accelSphere[3] = { rx, ry, rz };           // g, sphere frame, unsmoothed
      // Track gravity and check for shakes
      gravity.update(accelSphere, gyroSphereDps, dt);  // handles gaps and (re)seeding itself
      if (gravity.isInitialised &&
          shake.update(accelSphere, dt, millis(), gravity.gravityEstimate))
      {
        for (int i = 0; i < 3; i++) whenshaken_buffer[i] = shake.shakeAxisAtStart[i];
        shaken = true;
      }

      // Only activate smoothing filter if read gap is small enough
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

      // TODO PR: Old codebase code needs small refactor
      // polar and azimuth angles of the smoothed acceleration vector in the sensor frame
      // NOT rotational accellerations values. Maybe rename?
      t_acc = theta(x, y, z) * RAD_TO_DEG;
      p_acc = phi(x, y) * RAD_TO_DEG;
      if (p_acc < 0)
      {
        p_acc += 360;
      } // to bring it to [0,360) range

      if (!tappedrecorded && tapped)
      {
        tappedrecorded = true;
        if (gravity.isInitialised)
        {
          // Best estimate: unit length, no lag during rotation, tap impulse ignored by the trust gate.
          for (int i = 0; i < 3; i++) whentapped_buffer[i] = gravity.gravityEstimate[i];
        }
        else
        {
          // Tracker not valid yet (boot, long gap, or continuous motion): fall back to the
          // smoothed accelerometer vector, normalised to unit length like the tracker's output.
          float smoothedNorm = sqrtf(x*x + y*y + z*z);
          if (smoothedNorm > 1e-6f)
          {
            whentapped_buffer[0] = x / smoothedNorm;
            whentapped_buffer[1] = y / smoothedNorm;
            whentapped_buffer[2] = z / smoothedNorm;
          }
          // else: keep the previous buffer content rather than storing an invalid vector
        }
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

    bool applyPreparedState(uint32_t state)
    {
      BlochVector up(0, 0);
      BlochVector down(180, 0);
      if (state == 1)
      {
        this->clearStates();
        this->addState(up);
        this->addState(down);
        return true;
      }
      else if (state == 2)
      {
        this->clearStates();
        this->addState(down);
        this->addState(up);
        return true;
      }
      return false;
    }

    // Hacky solution for the colours as it gives away inner implementation of the class
    void displayCurrentStatesStatic(uint8_t colorOffset = 0)
    {
      this->clear();
      for (uint8_t i = 0; i < innerStateCount; i++)
      {
        const BlochVector &item = innerStates[i];
        const uint32_t &itemColour = stateColours[(i + colorOffset) % INNER_STATE_COUNT];

        this->setBloch_deg(item, itemColour);
      }
      this->show();
    }

    void displayCurrentStatesCycling()
    {
      if (innerStateCount == 0)
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