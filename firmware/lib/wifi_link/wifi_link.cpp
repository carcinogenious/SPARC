#include "wifi_link.h"

#include <WiFi.h>
#include <WiFiUdp.h>

namespace {
WiFiUDP udp;
IPAddress broadcastAddr;
bool ready = false;
}  // namespace

bool wifi_link::init() {
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(SSID, PASS)) {
        return false;
    }

    IPAddress ip = WiFi.softAPIP();
    // Broadcast to the /24 the SoftAP hands out (default 192.168.4.x).
    broadcastAddr = IPAddress(ip[0], ip[1], ip[2], 255);
    udp.begin(PORT);

    Serial.print(F("WiFi AP SSID:   ")); Serial.println(SSID);
    Serial.print(F("WiFi AP IP:     ")); Serial.println(ip);
    Serial.print(F("UDP telemetry:  port ")); Serial.println(PORT);
    ready = true;
    return true;
}

void wifi_link::send(const Packet& p) {
    if (!ready) return;

    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "{\"alt\":%.2f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
        "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
        "\"tof_r\":%.3f,\"tof_v\":%.3f,\"tilt\":%.1f}",
        p.altitude_m,
        p.accel_x, p.accel_y, p.accel_z,
        p.gyro_x, p.gyro_y, p.gyro_z,
        p.tof_range_m, p.tof_vertical_m, p.tilt_deg);
    if (n <= 0) return;

    udp.beginPacket(broadcastAddr, PORT);
    udp.write(reinterpret_cast<const uint8_t*>(buf), n);
    udp.endPacket();
}
