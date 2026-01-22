# CLAUDE.md - QBead Library

## Overview
QBead is an Arduino library for the QBead device - a Bloch sphere visualizer that represents quantum states using a spherical arrangement of RGB LEDs. It combines an IMU (accelerometer/gyroscope), NeoPixel LEDs, and Bluetooth Low Energy (BLE) for interactive quantum mechanics education.

## Hardware
- **Microcontroller**: Seeed XIAO nRF52840 Sense
- **IMU**: LSM6DS3 (I2C address 0x6A)
- **LEDs**: WS2812B NeoPixels - 6 sections x 12 legs = 72 LEDs in spherical arrangement
- **Board URL**: `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`

## Project Structure
```
src/Qbead.h          # Single-header library (all code in one file)
examples/
  BLE_reader/        # Basic BLE control of LED display
  IMU_reader/        # Motion-to-visualization (gravity orientation)
  BLE_bridge/        # Hybrid: IMU broadcast + BLE control
  Dynamical_Decoupling/  # Quantum decoherence simulation game
  Tap_to_Measure/    # Quantum measurement collapse simulation
```

## Key Classes

### BlochVector (src/Qbead.h:158-239)
Represents a point on the Bloch sphere (quantum state).
- Constructors: spherical angles (theta, phi) or Cartesian (x, y, z)
- `rotateAround(axis, angle)` - Rodrigues' rotation formula
- `centralAngle(other)` - Angular distance
- `innerProductAbs(other)` - Quantum probability amplitude

### Qbead (src/Qbead.h:266-698)
Main hardware control class (singleton for interrupt callbacks).
- `begin()` - Initialize hardware, IMU, BLE, interrupts
- `readIMU(bool print)` - Read accelerometer with low-pass filter
- `setBloch_deg(theta, phi, color)` - Display quantum state at LED position
- `setBloch_deg_smooth(theta, phi, color)` - Smooth interpolation between LEDs
- `wasTapped()` / `setupIMUTapDetection()` - Tap event handling via INT1 interrupt

### State Variables
- `t_acc, p_acc` - Theta/phi from IMU (gravity-based)
- `t_ble, p_ble, c_ble` - Theta/phi/color from BLE commands
- `x_whentapped, y_whentapped, z_whentapped` - Acceleration at tap time

## Utility Functions
- `color(r, g, b)` - Create RGB color uint32_t
- `colorWheel(pos)` - Hue wheel (0-255)
- `colorWheel_deg(angle)` - Hue by degrees (0-360)
- `scaleColor(brightness, color)` - Multiply color channels
- `innerProductAbs(v1, v2)` - Quantum probability |<v1|v2>|

## BLE Characteristics
5 characteristics (all read/write/notify):
1. Color - RGB (3 bytes)
2. Spherical - theta, phi (2 bytes, 0-255 scaled)
3. Acceleration - XYZ (3 floats, 12 bytes)
4. Tap acceleration - XYZ at tap time (3 floats, 12 bytes)

## Architecture Patterns
- **Single-header library**: All code in `Qbead.h` for simplicity
- **Non-blocking design**: No `delay()` - use `millis()` comparisons
- **Singleton pattern**: Static instance for interrupt callbacks
- **Hardware abstraction**: Decouples IMU input from LED output

## Development Guidelines
- Use named constants for magic numbers (QB_LEDPIN, QB_NSECTIONS, etc.)
- Low-pass filter timeconstant: 100ms (configurable)
- Tap detection uses LSM6DS3 INT1 pin with configurable threshold
- Max 2 simultaneous BLE peripheral connections

## Dependencies
- `Adafruit NeoPixel`
- `Seeed Arduino LSM6DS3`
- `bluefruit.h` (part of Seeed nRF52 board support)

## License
GNU GPL v3 - https://qbead.org
