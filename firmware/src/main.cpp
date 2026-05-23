// SPARC Flight Controller — Dual PID + TVC
// Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + OLED)
// See CLAUDE.md for full system spec

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <SSD1306Wire.h>
#include <LoRa.h>

// SPARC sensor drivers (firmware/lib/sensors/)
#include <bmp388.h>
#include <mpu6050.h>

// ── Pin Definitions (Heltec V3) ──
#define PIN_ARM_BTN       2     // Latching push button
#define PIN_BUZZER        12
#define PIN_LED           13
#define PIN_TVC_PITCH     26
#define PIN_TVC_YAW       33
#define PIN_THROTTLE      25
#define PIN_SD_CS         47
#define PIN_SD_MOSI       10
#define PIN_SD_MISO       11
#define PIN_SD_SCK        9
#define PIN_I2C_SDA       41
#define PIN_I2C_SCL       42
#define PIN_BATT_ADC      1

// ── Heltec V3 LoRa pins (internal, do not reassign) ──
#define LORA_SCK          9
#define LORA_MISO         11
#define LORA_MOSI         10
#define LORA_CS           8
#define LORA_RST          12
#define LORA_DIO1         14
#define LORA_BUSY         13

// ── OLED (on-board, I2C) ──
SSD1306Wire oled(0x3C, SDA_OLED, SCL_OLED);

// ── Servo Objects ──
Servo throttleServo;
Servo tvcPitchServo;
Servo tvcYawServo;

// ── Constants ──
const float TARGET_ALT = 1.524;       // 5 feet in meters
const float ASCENT_VEL = 1.5;         // m/s target ascent rate
const float DESCENT_VEL = -0.5;       // m/s target descent rate
const float TILT_ABORT_DEG = 30.0;    // safety cutoff
const float MAX_ALT = 10.0;           // runaway protection
const float TVC_CENTER = 90;          // servo center position
const float TVC_MAX_DEFLECT = 12;     // ±12° gimbal range

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

    // I2C for sensors (custom pins on Heltec V3)
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

    // OLED init
    oled.init();
    oled.flipScreenVertically();
    oled.setFont(ArialMT_Plain_10);
    oled.drawString(0, 0, "SPARC v3.0");
    oled.drawString(0, 14, "Initializing...");
    oled.display();

    Serial.println(F("SPARC Flight Controller v3.0"));
    Serial.println(F("Heltec WiFi LoRa 32 V3 — TVC enabled"));

    // ── Sensor Initialization ──
    // bmp388::init() also (re)starts the shared I2C bus, so I2C sensor
    // drivers added below can assume the bus is already up.
    bool bmpOk = bmp388::init(PIN_I2C_SDA, PIN_I2C_SCL);
    Serial.print(F("BMP388 altimeter: "));
    Serial.println(bmpOk ? F("OK") : F("FAIL"));
    // mpu6050::init() spends ~1 s capturing gyro static bias — keep the
    // rocket still through boot or attitude will drift on the pad.
    bool mpuOk = mpu6050::init(PIN_I2C_SDA, PIN_I2C_SCL);
    Serial.print(F("MPU6050 IMU:      "));
    Serial.println(mpuOk ? F("OK") : F("FAIL"));
    // TODO: bool tofOk = vl53l1x::init();   — ToF rangefinder (low alt)

    // Pre-flight sensor status on the OLED
    oled.clear();
    oled.drawString(0, 0, "SPARC v3.0");
    oled.drawString(0, 14, String("BMP388  ") + (bmpOk ? "OK" : "FAIL"));
    oled.drawString(0, 26, String("MPU6050 ") + (mpuOk ? "OK" : "FAIL"));
    // TODO: VL53L1X status at y=38
    oled.display();

    // TODO: init SD card on HSPI (PIN_SD_CS/MOSI/MISO/SCK)
    // TODO: init LoRa (SX1262, 915MHz)
    // TODO: set state = IDLE
}

void loop() {
    // 50Hz control loop (20ms period)
    static unsigned long lastLoop = 0;
    unsigned long now = millis();
    if (now - lastLoop < 20) return;
    lastLoop = now;

    // 1. Read all sensors (BMP388, MPU6050, VL53L1X)
    float altBaro = bmp388::readAltitude();   // meters AGL, NAN on I2C failure
    mpu6050::Reading imu = mpu6050::read();   // m/s² and rad/s, imu.ok on read
    // TODO: read VL53L1X (low-altitude range)

    // Bench-test scaffold: stream raw sensor values at ~2 Hz so each driver
    // can be verified standalone. Remove once fusion + logging consume them.
    static uint8_t dbgCount = 0;
    if (++dbgCount >= 25) {
        dbgCount = 0;
        Serial.print(F("alt="));
        Serial.print(isnan(altBaro) ? String("FAIL") : String(altBaro, 2) + "m");
        Serial.print(F("  accel(m/s2)="));
        if (imu.ok) {
            Serial.print(imu.accel_x, 2); Serial.print(',');
            Serial.print(imu.accel_y, 2); Serial.print(',');
            Serial.print(imu.accel_z, 2);
            Serial.print(F("  gyro(rad/s)="));
            Serial.print(imu.gyro_x, 3); Serial.print(',');
            Serial.print(imu.gyro_y, 3); Serial.print(',');
            Serial.print(imu.gyro_z, 3);
        } else {
            Serial.print(F("FAIL"));
        }
        Serial.println();
    }

    // TODO:
    // 2. Run sensor fusion (altitude + attitude)
    // 3. Run state machine transitions
    // 4. Run throttle PID (altitude/velocity → ball valve servo)
    // 5. Run TVC PID (pitch + roll → gimbal servos)
    // 6. Write servo outputs
    // 7. Log to SD card (CSV at 50Hz)
    // 8. Send telemetry via LoRa (10Hz, every 5th loop)
    // 9. Check safety cutoffs (tilt, alt, battery, sensor failure)
    // See CLAUDE.md for full architecture
}
