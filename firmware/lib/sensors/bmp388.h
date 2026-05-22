// BMP388 barometric altimeter driver — SPARC flight controller
// Wraps the Adafruit BMP3XX library. Reports altitude relative to the
// launch pad (AGL) in meters.
#pragma once

#include <Arduino.h>

namespace bmp388 {

// Initializes the BMP388 on the shared I2C bus and captures the
// ground-level pressure as the 0 m reference. Call once after power-on,
// before readAltitude(). Returns false if the sensor is not found.
bool init(uint8_t sdaPin = 41, uint8_t sclPin = 42);

// Returns altitude above the launch reference in meters.
// Returns NAN on an I2C read failure.
float readAltitude();

}  // namespace bmp388
