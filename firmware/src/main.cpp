// SPARC Flight Controller — Dual PID + TVC
// Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + OLED)
// See CLAUDE.md for full system spec

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

// SPARC sensor drivers (firmware/lib/sensors/)
#include <bmp388.h>
#include <mpu6050.h>
#include <tof.h>

// SPARC display module (firmware/lib/display/)
#include <display.h>

// SPARC WiFi telemetry (firmware/lib/wifi_link/)
#include <wifi_link.h>

// SPARC sensor fusion (firmware/lib/fusion/)
#include <attitude.h>

// ── Pin Definitions (Heltec V3 — header-accessible pins only) ──
// GPIO 6/7 are flash on the ESP32-S3FN8 module — never reassign them.
// GPIO 8-14 are the internal SX1262 LoRa bus, GPIO 26-32 are SPI flash,
// and GPIO 45/46 are strapping pins. Everything below is a safe, broken-out
// GPIO. The sensors share one I2C bus on GPIO 41 (SDA) / 42 (SCL).
#define PIN_ARM_BTN       2     // Latching push button
#define PIN_BUZZER        19
#define PIN_LED           20
#define PIN_TVC_PITCH     48    // MG90S #1 (gimbal pitch)
#define PIN_TVC_YAW       3     // MG90S #2 (gimbal yaw)
#define PIN_THROTTLE      47    // MG996R (ball valve)
#define PIN_I2C_SDA       41
#define PIN_I2C_SCL       42
#define PIN_BATT_ADC      1

// ── Servo Objects ──
Servo throttleServo;
Servo tvcPitchServo;
Servo tvcYawServo;

// ── Sensor presence flags ──
// Set once in setup(); gate the per-sensor reads in loop() so a sensor
// that wasn't found at boot is never polled. This is what lets a single
// plugged-in sensor stream on its own — polling a missing one would make
// its driver wait out the I2C timeout and stall the shared bus.
bool bmpOk = false;
bool mpuOk = false;
bool tofOk = false;

// ── Constants ──
const float TARGET_ALT = 1.524;       // 5 feet in meters
const float ASCENT_VEL = 1.5;         // m/s target ascent rate
const float DESCENT_VEL = -0.5;       // m/s target descent rate
const float TILT_ABORT_DEG = 30.0;    // safety cutoff
const float MAX_ALT = 10.0;           // runaway protection
const float TVC_CENTER = 90;          // servo center position
const float TVC_MAX_DEFLECT = 12;     // ±12° gimbal range
const float M_TO_IN = 39.3701f;       // meters → inches for the serial feed

// ── PID Gains (tune in MATLAB first, paste here) ──
// Altitude/Throttle PID
float Kp_alt = 18.0;
float Ki_alt = 8.0;
float Kd_alt = 25.0;

// Attitude/TVC PID (pitch and yaw use same gains)
float Kp_att = 22.0;
float Ki_att = 12.0;
float Kd_att = 18.0;

void setup() {
    Serial.begin(115200);

    // I2C for sensors (BMP388 + MPU6050 + VL53L1X) on GPIO 41/42.
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    // GPIO
    pinMode(PIN_ARM_BTN, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED, OUTPUT);

    // Servos (ESP32Servo uses LEDC channels internally)
    throttleServo.attach(PIN_THROTTLE);
    tvcPitchServo.attach(PIN_TVC_PITCH);
    tvcYawServo.attach(PIN_TVC_YAW);

    // Start safe: valve closed, TVC centered
    throttleServo.write(0);
    tvcPitchServo.write(TVC_CENTER);
    tvcYawServo.write(TVC_CENTER);

    // OLED init. display::init() pulses RST_OLED and brings up the SSD1306
    // on its own bus (Wire1 / SDA_OLED+SCL_OLED), kept separate from the
    // sensor bus on Wire so OLED traffic never stalls the PID loop.
    display::init();

    Serial.println(F("SPARC Flight Controller v3.0"));
    Serial.println(F("Heltec WiFi LoRa 32 V3 — TVC enabled"));

    // ── Sensor Initialization ──
    // bmp388::init() also (re)starts the shared I2C bus, so I2C sensor
    // drivers added below can assume the bus is already up.
    bmpOk = bmp388::init(PIN_I2C_SDA, PIN_I2C_SCL);
    Serial.print(F("BMP388 altimeter: "));
    Serial.println(bmpOk ? F("OK") : F("FAIL"));

    // Capture the barometric launch zero. Hold the rocket still on the pad:
    // we sample until the altitude reading settles (or 10 s elapses) and make
    // that the AGL reference, so baro= reads ~0 at rest regardless of weather
    // or site. OLED shows progress, and flags it if the pressure never settles.
    if (bmpOk) {
        display::message("Waiting for", "pressure...");
        bool baroStable = bmp388::captureBaseline(10000);
        Serial.print(F("BMP388 baseline:  "));
        Serial.println(baroStable ? F("stable") : F("UNSTABLE (10s timeout)"));
        if (!baroStable) {
            display::message("Pressure", "NOT stable!");
            delay(2000);
        }
    }
    // mpu6050::init() spends ~1 s capturing gyro static bias — keep the
    // rocket still through boot or attitude will drift on the pad.
    mpuOk = mpu6050::init(PIN_I2C_SDA, PIN_I2C_SCL);
    Serial.print(F("MPU6050 IMU:      "));
    Serial.println(mpuOk ? F("OK") : F("FAIL"));
    tofOk = tof::init(PIN_I2C_SDA, PIN_I2C_SCL);
    Serial.print(F("VL53L1X ToF:      "));
    Serial.println(tofOk ? F("OK") : F("FAIL"));

    // Capture the ToF launch zero (the resting standoff above the pad) the
    // same way as the barometer, so range_m reads ~0 at rest and tracks height
    // risen from launch. Hold the rocket still; OLED shows progress / failure.
    if (tofOk) {
        display::message("Waiting for", "ToF zero...");
        bool tofStable = tof::captureBaseline(10000);
        Serial.print(F("VL53L1X baseline: "));
        Serial.println(tofStable ? F("stable") : F("UNSTABLE (10s timeout)"));
        if (!tofStable) {
            display::message("ToF range", "NOT stable!");
            delay(2000);
        }
    }

    // Pre-flight sensor status on the OLED. Battery sense not wired yet,
    // so pass a placeholder until that lands.
    display::sensorStatus(bmpOk, mpuOk, tofOk, /* battV */ 0.0f);

    // ── WiFi Telemetry ──
    // SoftAP + UDP broadcast. Self-contained: a ground-station laptop
    // joins the AP and runs `nc -ul 4210` (or similar) to receive.
    bool wifiOk = wifi_link::init();
    Serial.print(F("WiFi telemetry:   "));
    Serial.println(wifiOk ? F("OK") : F("FAIL"));

    // TODO: set state = IDLE
}

// One telemetry field: "<value><unit>" with two decimals, or "FAIL" when
// the reading is unavailable (NAN). Keeps the live feed columns honest —
// a missing sensor reads FAIL instead of a misleading 0.
static String fmt(float v, const char* unit) {
    return isnan(v) ? String("FAIL") : String(v, 2) + unit;
}

void loop() {
    // 50Hz control loop (20ms period)
    static unsigned long lastLoop = 0;
    unsigned long now = millis();
    if (now - lastLoop < 20) return;
    float dt_s = (now - lastLoop) / 1000.0f;
    lastLoop = now;

    // 1. Read each sensor — but only if it answered at boot (see the flag
    //    comments above). A sensor that failed init() is skipped so its
    //    driver never blocks the loop or the shared bus for the others.
    float altBaro = bmpOk ? bmp388::readAltitude() : NAN;   // meters AGL

    mpu6050::Reading imu{};                                  // imu.ok = false
    if (mpuOk) imu = mpu6050::read();

    tof::Reading rng{};
    rng.range_m = rng.vertical_m = rng.tilt_rad = NAN;       // default: no data
    if (tofOk) {
        // Pass the IMU gravity vector through (zeros if the IMU is absent —
        // the raw slant range still comes back, just without the driver's
        // own tilt correction, which we don't use here anyway).
        rng = imu.ok ? tof::read(imu.accel_x, imu.accel_y, imu.accel_z)
                     : tof::read(0.0f, 0.0f, 0.0f);
    }

    // 1b. Attitude estimate (complementary filter): gyro for the fast,
    //     thrust-immune short term, accel as the gravity reference. Step it
    //     only on a fresh IMU sample; otherwise hold the last angles.
    static fusion::Attitude att{NAN, NAN};
    static bool attReady = false;
    if (imu.ok) {
        att = fusion::update(imu.accel_x, imu.accel_y, imu.accel_z,
                             imu.gyro_x, imu.gyro_y, dt_s);
        attReady = true;
    }

    // ── Live serial telemetry (~5 Hz) ──────────────────────────────
    // Each sensor prints independently (FAIL when absent). The fused
    // vertical altitude is a right triangle: the ToF measures the SLANT
    // range along the rocket's long axis (hypotenuse) and the IMU gives
    // the tilt off vertical (angle), so true height = range·cos(pitch).
    // Printed next to the barometer's independent altitude plus their
    // difference, so disagreement between the two is visible at a glance.
    static uint8_t dbgCount = 0;
    if (++dbgCount >= 10) {
        dbgCount = 0;

        float pitch  = attReady ? att.pitch_deg : NAN;                   // deg, signed
        float roll   = attReady ? att.roll_deg  : NAN;                   // deg, signed
        float baroIn = isnan(altBaro)     ? NAN : altBaro     * M_TO_IN;  // baro altitude
        float tofIn  = isnan(rng.range_m) ? NAN : rng.range_m * M_TO_IN;  // slant range
        // Tilt-correct the ToF slant range to true vertical with the fused
        // angles: the z-axis tilt off vertical gives cos(tilt) = cos(pitch)·cos(roll).
        float cosTilt = attReady ? cosf(pitch * DEG_TO_RAD) * cosf(roll * DEG_TO_RAD)
                                 : NAN;
        float vertIn = (isnan(cosTilt) || isnan(tofIn)) ? NAN : tofIn * cosTilt;
        float dIn    = (isnan(vertIn) || isnan(baroIn)) ? NAN : baroIn - vertIn;

        Serial.print(F("baro="));        Serial.print(fmt(baroIn, "in"));
        Serial.print(F("  pitch="));     Serial.print(fmt(pitch, "deg"));
        Serial.print(F("  roll="));      Serial.print(fmt(roll, "deg"));
        Serial.print(F("  tof="));       Serial.print(fmt(tofIn, "in"));
        Serial.print(F("  | vert="));    Serial.print(fmt(vertIn, "in"));
        Serial.print(F("  baro-vert=")); Serial.print(fmt(dIn, "in"));
        Serial.println();
    }

    // UDP telemetry broadcast at 10 Hz (every 5th 50 Hz loop). Best-effort:
    // drop on failure, no retries. JSON packet is human-readable so the
    // ground station can debug with `nc -ul 4210`.
    static uint8_t txCount = 0;
    if (++txCount >= 5) {
        txCount = 0;
        wifi_link::Packet pkt{};
        pkt.altitude_m     = isnan(altBaro) ? 0.0f : altBaro;
        pkt.accel_x        = imu.ok ? imu.accel_x : 0.0f;
        pkt.accel_y        = imu.ok ? imu.accel_y : 0.0f;
        pkt.accel_z        = imu.ok ? imu.accel_z : 0.0f;
        pkt.gyro_x         = imu.ok ? imu.gyro_x  : 0.0f;
        pkt.gyro_y         = imu.ok ? imu.gyro_y  : 0.0f;
        pkt.gyro_z         = imu.ok ? imu.gyro_z  : 0.0f;
        pkt.tof_range_m    = isnan(rng.range_m)    ? 0.0f : rng.range_m;
        pkt.tof_vertical_m = isnan(rng.vertical_m) ? 0.0f : rng.vertical_m;
        pkt.tilt_deg       = isnan(rng.tilt_rad)   ? 0.0f : rng.tilt_rad * 57.2958f;
        wifi_link::send(pkt);
    }

    // IDLE-state display refresh (~2 Hz). Once the state machine lands,
    // guard this on state == IDLE and switch to display::armed/blank/
    // flightSummary for the other states.
    static uint8_t idleCount = 0;
    if (++idleCount >= 25) {
        idleCount = 0;
        // Each row is an independent, live health check of one sensor — no
        // sensor's OK/FAIL depends on another's. BMP and MPU report their own
        // read() success; rng.sensor_ok is the ToF's own I2C liveness, kept
        // deliberately separate from rng.ok (the tilt-corrected vertical flag,
        // which needs the IMU and must never drive this light).
        display::sensorStatus(/* bmp */ !isnan(altBaro),
                              /* mpu */ imu.ok,
                              /* tof */ rng.sensor_ok,
                              /* batt */ 0.0f);
    }

    // TODO:
    // 2. Run sensor fusion (altitude + attitude)
    // 3. Run state machine transitions
    // 4. Run throttle PID (altitude/velocity → ball valve servo)
    // 5. Run TVC PID (pitch + roll → gimbal servos)
    // 6. Write servo outputs
    // 7. Check safety cutoffs (tilt, alt, battery, sensor failure)
    //    (data logging + telemetry already stream over WiFi/UDP above)
    // See CLAUDE.md for full architecture
}
