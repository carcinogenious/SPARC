// MPU6050 6-axis IMU driver — SPARC flight controller
// Wraps the Adafruit MPU6050 library. Returns calibrated accel (m/s²)
// and gyro (rad/s) for the attitude complementary filter driving TVC.
#pragma once

#include <Arduino.h>

namespace mpu6050 {

struct Reading {
    float accel_x, accel_y, accel_z;   // m/s², gravity included
    float gyro_x, gyro_y, gyro_z;      // rad/s, static bias subtracted
    bool ok;                            // false on I2C read failure
};

// Initializes the MPU6050 on the shared I2C bus and captures the gyro
// static bias by averaging ~1 s of readings — the rocket must be at rest
// on the pad during boot. Returns false if the sensor is not found.
bool init(uint8_t sdaPin = 7, uint8_t sclPin = 6);

// Single I2C transaction reading all 6 axes. ok=false on read failure.
Reading read();

}  // namespace mpu6050
