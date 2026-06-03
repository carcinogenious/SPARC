// OLED display module — SPARC flight controller
// Wraps the Heltec V3's on-board SSD1306 (128x64 @ 0x3C) on its dedicated
// I2C bus (Wire1 / pins SDA_OLED, SCL_OLED) so OLED traffic does not
// share the sensor bus on Wire. Each draw function calls display()
// internally — callers never touch the SSD1306Wire object directly.
#pragma once

#include <Arduino.h>

namespace display {

// Resets the OLED (Heltec V3 requires a manual RST pulse on GPIO 21
// before init) and brings up the SSD1306 in flipped orientation.
void init();

// Generic centered two-line message for transient boot states (e.g. the
// barometer baseline capture: "Waiting for" / "pressure..."). Pass an empty
// second line for a single centered line.
void message(const String& line1, const String& line2 = String());

// Pre-launch sensor/battery readout for the IDLE state. One line per
// sensor + a final line for battery voltage.
void sensorStatus(bool bmpOk, bool mpuOk, bool tofOk, float battV);

// ARMED state: large "ARMED" banner with a countdown timer below.
void armed(int countdownSec);

// Post-landing (SAFE state) flight summary: peak altitude, hover time,
// max tilt, landing speed, CO₂ used.
void flightSummary(float peakAlt, float hoverTime, float maxTilt,
                   float landingSpeed, float co2Used);

// Clears the screen. Called during LAUNCH..LANDED to free CPU cycles
// for the 50 Hz PID loop.
void blank();

}  // namespace display
