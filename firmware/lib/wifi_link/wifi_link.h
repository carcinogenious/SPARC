// WiFi telemetry link — SPARC flight controller
// Brings up the ESP32-S3 in SoftAP mode and broadcasts a compact JSON
// telemetry packet over UDP. Self-contained: no router or credentials
// needed in the field. Any device that joins the AP can `nc -ul 4210`
// to receive the live stream.
//
// Named wifi_link.h (not wifi.h) to avoid a case-insensitive filesystem
// collision with Arduino's WiFi.h.
#pragma once

#include <Arduino.h>

namespace wifi_link {

// Fixed AP credentials and UDP port. Documented here so ground-station
// code can hard-code them without reaching into the implementation.
constexpr const char* SSID = "SPARC-Telemetry";
constexpr const char* PASS = "sparc1234";
constexpr uint16_t    PORT = 4210;

struct Packet {
    float altitude_m;                       // BMP388 AGL
    float accel_x, accel_y, accel_z;        // m/s²
    float gyro_x, gyro_y, gyro_z;           // rad/s
    float tof_range_m;                      // VL53L1X slant range
    float tof_vertical_m;                   // tilt-corrected vertical altitude
    float tilt_deg;                         // body tilt from gravity
};

// Brings up the SoftAP and binds the UDP socket. Returns false on
// softAP() failure (extremely rare). Logs SSID / IP / port to Serial.
bool init();

// Broadcasts `p` as a single UDP datagram to the AP's /24 broadcast
// address. Non-blocking, no retries — drop on failure is fine because
// telemetry is best-effort.
void send(const Packet& p);

}  // namespace wifi_link
