// VL53L1X time-of-flight rangefinder driver — SPARC flight controller
// Wraps the Pololu VL53L1X library. Mounted facing downward; provides the
// precise low-altitude reading used by the fusion filter when the rocket
// is close to the ground (< 4 m) and roughly upright.
//
// Named `tof.h` (not `vl53l1x.h`) to avoid a case-insensitive filesystem
// collision with the Pololu library's `VL53L1X.h`.
#pragma once

#include <Arduino.h>

namespace tof {

struct Reading {
    float range_m;          // slant range from sensor face along the rocket body z-axis
    float vertical_m;       // tilt-corrected altitude above the ground (range * cos(tilt))
    float tilt_rad;         // tilt angle used for the correction (0 = perfectly upright)
    bool ok;                 // false on I2C timeout, invalid status, or out-of-range
};

// Initializes the VL53L1X on the shared I2C bus in long-range continuous
// mode (~4 m max) at the 50 Hz control-loop rate. Returns false if the
// sensor is not found.
bool init(uint8_t sdaPin = 41, uint8_t sclPin = 42);

// Reads the latest slant range and combines it with the IMU's gravity
// vector (accel_x/y/z, m/s²) to compute the true vertical altitude.
// The rangefinder points along the rocket's body z-axis; tilt is the
// angle between that axis and gravity. Non-blocking: if no new ToF
// sample is ready, returns the previous slant range with a fresh
// tilt correction.
Reading read(float accel_x, float accel_y, float accel_z);

}  // namespace tof
