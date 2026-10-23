/**
 * @file Qbead.h
 * @brief Public interface for the Qbead LED sphere, IMU, and BLE controller.
 */
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
#include "internal/QbeadPixels.h"
#include "internal/ShakeDetector.h"
#include "internal/GravityTracker.h"
#include "internal/ProbabilityDisplay.h"

namespace Qbead
{
  /**
   * @brief Controls the Qbead display, motion sensing, stored states, and BLE.
   * @details Call begin() once, then readIMU() regularly to update motion data and
   * capture tap/shake events. LED drawing changes the pixel buffer; call show()
   * to transmit it, unless a display helper already does so.
   * @warning Only one instance may serve as the active interrupt callback target.
   * @note Public fields and original method names are retained for compatibility.
   */
  class Qbead : public BandPainter
  {
  public:
    /**
     * @brief Constructs the controller without initializing hardware.
     * @param pin00 NeoPixel data pin.
     * @param pixelconfig NeoPixel pixel type and timing configuration.
     * @param nsections Number of polar intervals between the two poles.
     * @param nlegs Number of azimuthal LED legs.
     * @param imu_addr IMU I2C address.
     * @param ix Chip-frame axis index mapped to sphere-frame X (0, 1, or 2).
     * @param iy Chip-frame axis index mapped to sphere-frame Y (0, 1, or 2).
     * @param iz Chip-frame axis index mapped to sphere-frame Z (0, 1, or 2).
     * @param sx Whether to reverse the mapped X axis.
     * @param sy Whether to reverse the mapped Y axis.
     * @param sz Whether to reverse the mapped Z axis.
     * @pre Geometry must be compatible with the pixel-index mapping and fit in
     * the stored uint8_t geometry fields. Angular spacings use integer division;
     * choose nsections and nlegs so the resulting spacings are nonzero.
     */
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
          const bool sz = QB_SZ);

    /** @brief Active instance used by callbacks that cannot carry context. */
    static Qbead *callbackTarget;

    LSM6DS3 imu;                                /**< IMU driver. */
    QbeadIndexed_NeoPixelStrip pixels;          /**< Indexed LED strip and pixel buffer. */
    ShakeDetector shake;                        /**< Software shake detector. */
    GravityTracker gravity;                     /**< Gyroscope-assisted gravity tracker. */
    ProbabilityDisplay prMode;                  /**< Probability-band display helper. */
    BLEManager::BLEManager ble;                 /**< BLE communication manager. */
    BlochVector innerStates[INNER_STATE_COUNT]; /**< Fixed-capacity state storage. */
    uint8_t innerStateCount = 0;                /**< Number of occupied state entries. */

    const uint8_t nsections;                        /**< Number of polar intervals. */
    const uint8_t nlegs;                            /**< Number of azimuthal legs. */
    const uint8_t theta_quant;                      /**< Integer polar spacing, in degrees. */
    const uint8_t phi_quant;                        /**< Integer azimuthal spacing, in degrees. */
    const uint8_t ix, iy, iz;                       /**< Chip-to-sphere axis indices. */
    const bool sx, sy, sz;                          /**< Mapped-axis sign reversals. */
    float read_buffer[3];                           /**< Smoothed sphere-frame acceleration in g after readIMU(). */
    float whentapped_buffer[3] = {0.f, 0.f, 1.f};   /**< Buffered unit tap direction in the sphere frame. */
    float whenshaken_buffer[3] = {0.f, 0.f, 1.f};   /**< Unit shake axis at the start of the last shake. */
    float gyroBiasDps[3] = {0.f, 0.f, 0.f};         /**< Learned chip-frame gyro bias, in degrees per second. */
    float x_whentapped, y_whentapped, z_whentapped; /**< Tap direction copied by wasTapped(); undefined until first consumed tap. */
    float x_whenshaken, y_whenshaken, z_whenshaken; /**< Shake axis copied by wasShaken(); undefined until first consumed shake. */
    float theta_accvec = 0.f, phi_accvec = 0.f;     /**< Display angles of smoothed acceleration, in degrees. */
    volatile bool tapped = false;                   /**< Tap interrupt pending. */
    volatile bool tappedrecorded = false;           /**< Tap direction captured by readIMU(). */
    bool shaken = false;                            /**< Shake event pending consumption. */
    float x = 0.f, y = 0.f, z = 0.f;                /**< Smoothed sphere-frame acceleration, in g. */
    float rx = 0.f, ry = 0.f, rz = 0.f;             /**< Unsmoothed sphere-frame acceleration, in g. */
    uint32_t T_imu = 0;                             /**< Previous IMU update timestamp, in microseconds. */

    uint32_t stateColours[INNER_STATE_COUNT] = {
        color(0, 0, 255),
        color(255, 0, 0),
        color(0, 255, 0),
        color(255, 255, 0),
        color(255, 0, 255),
        color(255, 128, 0)}; /**< State palette: blue, red, green, yellow, magenta, orange. */
    uint32_t cyclingIndex = 0;              /**< Index of the next state to display while cycling. */
    uint32_t lastChange = 0;                /**< Last cycling update timestamp, in milliseconds. */
    uint32_t accelSmoothingTimeUs = 100000; /**< Acceleration smoothing time; zero disables smoothing. */
    bool hasAccelSample = false;            /**< Whether an acceleration sample has been processed. */

    bool localEntangleRequestPending = false;       /**< Whether a local request is awaiting a match. */
    uint32_t localEntangleRequestStartedAtMs = 0;   /**< Local request timestamp, in milliseconds. */
    bool remoteEntangleRequestPending = false;      /**< Whether a remote request is awaiting a match. */
    uint32_t remoteEntangleRequestReceivedAtMs = 0; /**< Remote request timestamp, in milliseconds. */

    /**
     * @brief Prepares the IMU settings that imu.begin() writes to the device.
     * @param gyroOn Whether to enable the gyroscope; defaults to true.
     * @warning readIMU() currently expects gyroscope data even if this is false.
     * @note Call before imu.begin().
     */
    void preConfigIMU_Settings(bool gyroOn = true);

    /** @brief Configures the accelerometer low-pass filter after imu.begin(). */
    void postConfigIMU_Filters();

    /**
     * @brief Configures single-tap detection and attaches the INT1 interrupt.
     * @pre The IMU is initialized and callbackTarget identifies the active instance.
     */
    void postConfigIMU_TapDetection();

    /**
     * @brief Initializes Serial, LEDs, IMU, tap interrupts, BLE, and the RNG.
     * @details Waits up to 15 seconds for Serial. Logs IMU initialization failure
     * but continues with the remaining initialization steps.
     */
    void begin();

    /** @brief Clears the pixel buffer without transmitting it. */
    void clear();
    /** @brief Transmits the pixel buffer to the LEDs. */
    void show();

    /**
     * @brief Sets a logical leg pixel in the pixel buffer.
     * @param leg Azimuthal leg index.
     * @param pixel Polar index from 0 through nsections, including both poles.
     * @param color Packed pixel color.
     * @pre Indices must be valid for computePixelIndex().
     */
    void setLegPixelColor(int leg, int pixel, uint32_t color);

    /**
     * @brief Sets all pixels in a polar band without transmitting the buffer.
     * @param band Polar band index from 0 through nsections.
     * @param color Packed pixel color.
     * @param invert Whether to replace band with nsections - band.
     * @note Each pole is represented by a single pixel.
     */
    void setBandColor(int band, uint32_t color, bool invert = true) override;

    /**
     * @brief Reads a logical pixel's buffered color.
     * @param leg Azimuthal leg index.
     * @param pixel Polar pixel index.
     * @return Packed buffered color.
     */
    uint32_t getLegPixelColor(int leg, int pixel);

    /**
     * @brief Adds a color to a logical pixel using addColor().
     * @param leg Azimuthal leg index.
     * @param pixel Polar pixel index.
     * @param c0 Packed color to add.
     */
    void addLegPixelColor(int leg, int pixel, uint32_t c0);

    /** @brief Sets strip brightness. @param b Brightness from 0 to 255. */
    void setBrightness(uint8_t b);

    /**
     * @brief Draws a Bloch vector at the nearest discrete display position.
     * @param state State whose theta and phi are expressed in degrees.
     * @param color Packed pixel color.
     */
    void setBloch_deg(const BlochVector &state, uint32_t color);

    /**
     * @brief Draws an angular position at the nearest discrete display position.
     * @param theta Polar angle in degrees, normally 0 to 180.
     * @param phi Azimuth in degrees, normally 0 to 360.
     * @param color Packed pixel color.
     * @note This overload does not validate the input angles.
     */
    void setBloch_deg(float theta, float phi, uint32_t color);

    /**
     * @brief Draws a state using squared weights on neighboring polar pixels.
     * @param state State whose theta and phi are expressed in degrees.
     * @param color Packed pixel color.
     * @note Azimuth is rounded to a leg; it is not interpolated.
     */
    void setBloch_deg_smooth(const BlochVector &state, uint32_t color);

    /**
     * @brief Draws angles using squared weights on neighboring polar pixels.
     * @param theta Polar angle in degrees.
     * @param phi Azimuth in degrees.
     * @param c Packed pixel color.
     * @details Returns without drawing if checkThetaAndPhi() rejects the angles.
     * Writes, rather than adds, the two weighted colors to the pixel buffer.
     */
    void setBloch_deg_smooth(float theta, float phi, uint32_t c);

    /** @brief Runs the blocking discrete and smooth LED display tests. */
    void testPixels();

    /**
     * @brief Consumes a tap whose direction has been captured by readIMU().
     * @return True when a recorded tap was consumed; otherwise false.
     * @details Copies the buffered direction to x_whentapped, y_whentapped,
     * and z_whentapped. A pending but not yet recorded tap is not consumed.
     */
    bool wasTapped();

    /**
     * @brief Consumes a pending shake and copies its buffered axis.
     * @return True when a shake was consumed; otherwise false.
     */
    bool wasShaken();

    /**
     * @brief Matches local and remote entanglement requests within their timeout.
     * @param localTrigger True to queue a local request and send it over BLE.
     * @param state Prepared-state identifier passed to applyPreparedState().
     * @return Result of applyPreparedState() when requests match; otherwise false.
     * @details Poll regularly to receive requests and expire pending ones.
     * Both requests are consumed on a match, even if state is unsupported.
     */
    bool entangle(bool localTrigger, uint32_t state);

    /**
     * @brief Retrieves a packet through the BLE manager's takePacket() interface.
     * @return Retrieved packet, or a None command with zero-valued fields.
     */
    BLEManager::DataPacket takeLatestPacket();

    /**
     * @brief Marks a tap as pending from the IMU interrupt context.
     * @pre callbackTarget must point to a live Qbead instance.
     * @note Direction sampling is deferred to readIMU(); no IMU read occurs here.
     */
    static void tap_isr();

    /**
     * @brief Updates motion data, gyro bias, gravity, gestures, and display angles.
     * @param print Whether to emit tab-separated diagnostic data to Serial.
     * @details Reads acceleration in g and gyroscope rates in degrees per second,
     * maps them to sphere coordinates, updates the gravity/shake trackers, and
     * smooths acceleration. Saves the smoothed acceleration in read_buffer.
     * A pending tap captures the gravity estimate, or a normalized smoothed
     * acceleration fallback if the gravity estimate is unavailable.
     */
    void readIMU(bool print = true);

    /**
     * @brief Checks whether a stored state is within angular tolerances.
     * @param state Candidate state, with angles in degrees.
     * @param thetaTolerance Maximum absolute polar difference, in degrees.
     * @param phiTolerance Maximum wrapped azimuthal difference, in degrees.
     * @return True if a stored state satisfies both tolerances.
     * @pre Azimuths are normalized so the wrapped-difference calculation applies.
     * @note Compares coordinates, not the central angle between Bloch vectors.
     */
    bool hasState(const BlochVector &state,
                  float thetaTolerance = 1.0f,
                  float phiTolerance = 1.0f) const;

    /**
     * @brief Adds a theta/phi copy if not already present and capacity remains.
     * @param state Source state; retained as non-const for API compatibility.
     * @note Duplicate checking uses hasState() with its default tolerances.
     */
    void addState(BlochVector &state);

    /** @brief Empties state storage logically and resets the cycling index. */
    void clearStates();

    /**
     * @brief Replaces stored states with a supported prepared arrangement.
     * @param state 1 stores up then down; 2 stores down then up.
     * @return True for a supported identifier; false otherwise.
     * @note Unsupported identifiers leave stored states unchanged.
     */
    bool applyPreparedState(uint32_t state);

    /**
     * @brief Clears, draws all stored states, and transmits the pixel buffer.
     * @param colorOffset Offset into the cyclic state color palette.
     */
    void displayCurrentStatesStatic(uint8_t colorOffset = 0);

    /**
     * @brief Displays the next stored state when CYCLING_TIME has elapsed.
     * @details Nonblocking; call repeatedly. Does nothing when storage is empty.
     */
    void displayCurrentStatesCycling();

    /**
     * @brief Retrieves the local Bluetooth address.
     * @param[out] localAddress Writable buffer of six bytes passed to ble.getAddr().
     * @note Original spelling retained for source compatibility.
     */
    void getBluetoothAddres(uint8_t localAddress[6]);

    /**
     * @brief Forwards a data packet to the BLE manager.
     * @param type Command type.
     * @param value Command-specific value.
     * @param theta Command-specific polar-angle field.
     * @param phi Command-specific azimuthal-angle field.
     */
    void sendData(BLEManager::CommandType type, uint32_t value,
                  uint32_t theta, uint32_t phi);
  };
} // namespace Qbead

#endif // QBEAD_H
