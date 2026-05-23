#include "tof.h"

#include <Wire.h>
#include <VL53L1X.h>
#include <math.h>

namespace {
::VL53L1X sensor;
float lastRange_m = NAN;
}  // namespace

bool tof::init(uint8_t sdaPin, uint8_t sclPin) {
    Wire.begin(sdaPin, sclPin);

    sensor.setBus(&Wire);
    sensor.setTimeout(100);
    if (!sensor.init()) {
        return false;
    }

    // Long mode reaches the full ~4 m we care about for low-altitude fusion.
    // 20 ms timing budget + 20 ms inter-measurement period gives the 50 Hz
    // sample rate the control loop expects.
    sensor.setDistanceMode(::VL53L1X::Long);
    sensor.setMeasurementTimingBudget(20000);
    sensor.startContinuous(20);
    return true;
}

tof::Reading tof::read(float accel_x, float accel_y, float accel_z) {
    Reading r{};

    // Non-blocking: only consume a new sample if one is ready, otherwise
    // recompute the tilt correction on the last-known slant range so the
    // loop keeps its 50 Hz cadence.
    if (sensor.dataReady()) {
        uint16_t mm = sensor.read(false);
        if (sensor.timeoutOccurred() || sensor.ranging_data.range_status != ::VL53L1X::RangeValid) {
            r.range_m = lastRange_m;
            r.vertical_m = NAN;
            r.tilt_rad = NAN;
            r.ok = false;
            return r;
        }
        lastRange_m = mm / 1000.0f;
    }

    r.range_m = lastRange_m;

    // Tilt from the gravity vector: the body z-axis (which the ToF points
    // along, downward) is the rocket frame. When stationary, accel = -g in
    // the inertial frame, so its projection onto the body z-axis directly
    // gives g·cos(tilt). |accel_z| / |accel| is therefore cos(tilt) without
    // needing pitch/roll Euler angles or a square root in the denominator.
    //
    // Caveat: during powered ascent the accel vector includes thrust, so
    // this is most accurate at hover/near-hover. The fusion filter will
    // replace this with a gyro-integrated attitude estimate later.
    float a_mag = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    if (a_mag < 1.0f || isnan(lastRange_m)) {
        // Free-fall or no valid range yet — cannot trust the correction.
        r.vertical_m = NAN;
        r.tilt_rad = NAN;
        r.ok = false;
        return r;
    }

    float cos_tilt = fabsf(accel_z) / a_mag;
    if (cos_tilt > 1.0f) cos_tilt = 1.0f;       // guard against FP rounding
    r.tilt_rad = acosf(cos_tilt);
    r.vertical_m = lastRange_m * cos_tilt;
    r.ok = true;
    return r;
}
