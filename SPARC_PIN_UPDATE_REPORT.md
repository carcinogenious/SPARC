# SPARC Hardware & Pin Update Report
## Send this entire file to Claude Code as context for the next task.

---

## CRITICAL: Why the BMP388 was not working

The BMP388 sensor failed to communicate because of two incorrect configurations:

### Problem 1: Wrong GPIO pins for I2C
Previous code used `Wire.begin(7, 6)` for the I2C bus. On the Heltec WiFi LoRa 32 V3 (ESP32-S3), **GPIO 6 and GPIO 7 are hardwired to the board's internal SPI Flash memory.** Calling `Wire.begin(7, 6)` immediately disconnects the flash, freezing/bricking the MCU until reset. The I2C bus never actually initialized.

**Fix:** The correct and ONLY safe I2C pins on this board are `Wire.begin(41, 42)` — GPIO 41 (SDA) and GPIO 42 (SCL). These are the board's primary external I2C lines.

### Problem 2: Wrong CS pin logic for BMP388
Previous wiring had BMP388 CS pin tied to 3.3V (HIGH). This is WRONG for the CJMCU-388 / BMP388 breakout board. **CS must be tied to GND to disable SPI mode and activate I2C communication.** With CS HIGH, the chip stays in SPI mode and ignores all I2C traffic on SDI/SCK.

**Fix:** BMP388 CS pin → GND (not 3.3V). SDO pin → GND (sets I2C address to 0x76).

---

## Heltec WiFi LoRa 32 V3 — Confirmed GPIO Map

### Safe, fully unrestricted GPIOs:
- GPIO 41 → Primary external SDA (I2C)
- GPIO 42 → Primary external SCL (I2C)
- GPIO 1 → General purpose (also UART0_TX). Has ADC capability.
- GPIO 2 → General purpose (also UART0_RX)
- GPIO 3 → General purpose
- GPIO 47 → General purpose
- GPIO 48 → General purpose

### Now available (LoRa disabled):
- GPIO 19 → Was shared with SX1262 LoRa SPI. Safe when LoRa not activated.
- GPIO 20 → Was shared with SX1262 LoRa SPI. Safe when LoRa not activated.
- GPIO 26 → Was shared with SX1262 LoRa SPI. Safe when LoRa not activated.

### NEVER USE:
- GPIO 6, 7 → Hardwired to SPI Flash. Will freeze/brick the board.
- GPIO 4, 5 → Physical header slots are unrouted. Dead pins. No connection.
- GPIO 45, 46 → Strapping pins. Will prevent boot/upload.
- GPIO 0 → Boot/PRG button. Avoid for external hardware.
- GPIO 8, 9, 10, 11, 12, 13, 14 → Internal LoRa SPI. Not on headers.
- GPIO 17, 18 → Internal OLED display. Not on headers. (GPIO 18 also controls Vext power.)

---

## Updated Pin Assignments (FINAL)

```cpp
// I2C bus (all sensors share this)
#define PIN_I2C_SDA       41
#define PIN_I2C_SCL       42

// Servos
#define PIN_THROTTLE      47    // MG996R → ball valve
#define PIN_TVC_PITCH     48    // MG90S #1 → gimbal pitch
#define PIN_TVC_YAW       3     // MG90S #2 → gimbal yaw

// User interface
#define PIN_ARM_BTN       2     // Latching push button, INPUT_PULLUP
#define PIN_BUZZER        19    // Active buzzer module
#define PIN_LED           20    // Status LED (optional)

// Analog
#define PIN_BATT_ADC      1     // Voltage divider from 2S LiPo (10kΩ+10kΩ)

// Spare
// GPIO 26 — available if needed
```

---

## Architecture Changes

### Removed: SD Card Module
- The HiLetgo Micro SD TF Card Reader required 4 SPI pins (CS, MOSI, MISO, SCK). With only 10 usable GPIOs total, there are not enough pins for SD + 3 servos + button + buzzer + battery ADC.
- **Replacement:** All flight data logging is done over WiFi. The ESP32 creates a WiFi access point or connects to a local network. Flight data is streamed to a laptop running a simple receiver script. This also enables real-time telemetry during flight without pulling a card after landing.

### Removed: LoRa Radio
- The SX1262 LoRa radio is built into the Heltec board but activating it locks GPIO 19, 20, and 26 (its internal SPI bus). These pins are needed for buzzer, LED, and spare.
- **Replacement:** WiFi handles all telemetry. Range is ~30-100ft which is sufficient for 5ft hover tests in a backyard. If longer range is needed later, LoRa can be re-enabled by reassigning buzzer/LED to different pins or using a GPIO expander.

### Changed: BMP388 Wiring
- CS pin: was 3.3V, now GND (activates I2C mode)
- SDO pin: remains GND (sets address 0x76)
- SDI (SDA): was GPIO 7, now GPIO 41
- SCK (SCL): was GPIO 6, now GPIO 42

### Changed: All Servo Pins
- MG996R throttle: was GPIO 5/25, now GPIO 47
- MG90S TVC pitch: was GPIO 4/26, now GPIO 48  
- MG90S TVC yaw: was GPIO 3/33, now GPIO 3 (unchanged, was correct)

### Changed: Peripheral Pins
- Arm button: was GPIO 2, remains GPIO 2 (unchanged)
- Buzzer: was GPIO 38/12, now GPIO 19
- LED: was GPIO 37/13, now GPIO 20
- Battery ADC: was GPIO 1, remains GPIO 1 (unchanged)

---

## Updated Sensor Wiring (confirmed working)

### BMP388 (I2C mode):
- VIN → 3.3V
- GND → GND
- SDI → GPIO 41 (this is the SDA line)
- SCK → GPIO 42 (this is the SCL line)
- CS → GND (MUST be GND to enable I2C mode)
- SDO → GND (sets address to 0x76)
- INT → unconnected
- 3Vo → unconnected

### GY-521 MPU6050:
- VCC → 3.3V
- GND → GND
- SDA → GPIO 41
- SCL → GPIO 42
- AD0 → GND (sets address to 0x68)
- INT → unconnected
- XDA → unconnected
- XCL → unconnected

### TOF400C VL53L1X:
- VIN → 3.3V
- GND → GND
- SDA → GPIO 41
- SCL → GPIO 42
- SHUT → unconnected
- INT → unconnected

### OLED (built-in, no external wiring):
- Uses internal SDA_OLED, SCL_OLED, RST_OLED defines from Heltec board support package
- Address: 0x3C
- Shares the internal I2C bus, NOT the external GPIO 41/42 bus

---

## Updated platformio.ini

```ini
[env:heltec_wifi_lora_32_V3]
platform = espressif32
board = heltec_wifi_lora_32_V3
framework = arduino
monitor_speed = 115200
board_build.partitions = huge_app.csv
lib_deps =
    adafruit/Adafruit BMP3XX Library
    adafruit/Adafruit MPU6050
    adafruit/Adafruit Unified Sensor
    pololu/VL53L1X
    madhephaestus/ESP32Servo
    thingpulse/ESP8266 and ESP32 OLED driver for SSD1306 displays
```

Note: `arduino-libraries/SD` and `sandeepmistry/LoRa` have been REMOVED from lib_deps. WiFi is built into the ESP32 framework — no extra library needed.

---

## What Claude Code needs to do now

1. **Update every file** in the project to use the corrected pin definitions above.
2. **Search for and replace** any reference to GPIO 4, 5, 6, 7, 9, 10, 11, 25, 33, 34, 38, or any other pin NOT in the approved list above.
3. **Remove all SD card code** — delete the SD logger module and any SD includes/references from main.cpp and other files.
4. **Remove all LoRa code** — delete any LoRa includes, init, or transmit code.
5. **Add WiFi telemetry stub** — create a basic WiFi AP mode function that will eventually stream flight data over UDP. For now, just init WiFi in AP mode with SSID "SPARC" and print the IP to serial and OLED.
6. **Update CLAUDE.md** pin table and remove SD/LoRa references.
7. **Rebuild with `pio run`** and confirm clean compilation.
