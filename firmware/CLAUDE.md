# SPARC Flight Controller

## Project Overview
SPARC (Student Propulsive Autonomous Rocketry Challenge) is a CO₂-powered rocket with proportional thrust control AND 2-axis Thrust Vector Control (TVC) for autonomous hover at ~5 feet and propulsive soft landing. The rocket maintains vertical orientation actively via TVC gimbal — not passively via fins alone.

## Architecture Summary
- **No pressure regulator** — full cartridge pressure (~850 psi) feeds directly into a 1000psi-rated ball valve
- **Proportional throttle** — MG996R servo actuates a stainless ball valve for 0-100% flow control
- **2-axis TVC** — Nozzle mounted in a 3D-printed gimbal, driven by two MG90S servos via pushrods, connected to the valve by a flexible braided PTFE hose
- **Dual PID** — One loop for altitude/throttle, one for attitude/TVC
- **Full sensor suite** — BMP388 + MPU6050 + VL53L1X + SD logging + LoRa telemetry

## Hardware

### Microcontroller
- **Heltec WiFi LoRa 32 V3** (ESP32-S3, 240 MHz dual-core, 8MB flash, 320KB SRAM)
- Built-in SX1262 LoRa radio (915MHz) — no separate telemetry module needed
- Built-in 0.96" OLED display — pre-flight status, sensor checks, state display on pad
- Built-in WiFi + Bluetooth — firmware updates, ground station data download
- 3.3V logic — all sensors are compatible
- Abundant GPIO + hardware PWM channels for 3 servos
- Use ESP32Servo library (not arduino Servo)

### Sensors (all on I2C bus)
- **BMP388** (addr 0x76 or 0x77): Barometric altitude, ±0.5m relative accuracy, up to 200Hz ODR
- **GY-521 MPU6050** (addr 0x68): 6-axis IMU (accel ±8g, gyro ±2000°/s). Primary attitude source for TVC.
- **TOF400C VL53L1X** (addr 0x29): Time-of-flight laser rangefinder, ±1mm accuracy to 4m, 50Hz max. Mount facing downward.

### Telemetry (built into flight computer)
- **SX1262 LoRa** (on-board, 915MHz): Real-time downlink of altitude, attitude, state, servo positions during flight. Ground station = second Heltec board or any SX1262 receiver.
- **0.96" OLED** (on-board, I2C addr 0x3C): Shows flight state, sensor status, battery voltage, arm status on the pad before launch. Blank during flight to save power.

### Propulsion
- **88g CO₂ cartridge** (~850 psi, M16×1.5 male thread)
- **MCS piercing adapter** with on/off valve (M16×1.5 female → G1/2 male ASA, built-in piercing pin)
- **Metalwork SS adapter** (G1/2 female → 1/4" NPT male, 304 SS, 2320 psi rated)
- **SS reducer** (1/4" NPT female → 1/8" NPT male)
- **ARITA 1/8" NPT stainless ball valve** (1000 psi rated) — the throttle
- **Earl's Speed-Flex 6" braided PTFE hose** (1/8" NPT M-F) — flexible link allowing nozzle to gimbal
- **Brass pipe cap nozzle** (1/8" NPT, drilled 2.0-2.5mm orifice)
- **NO REGULATOR** — full cartridge pressure feeds through. The ball valve and all upstream fittings are rated for 1000+ psi.

### Thrust Vector Control
- **3D-printed 2-axis gimbal** (K-9 style or custom design) — holds the nozzle, pivots ±12° in pitch and yaw
- **2× MG90S servos** (metal gear, ~2 kg·cm torque) — drive the gimbal via steel pushrods
- **1.2mm steel pushrods** — connect servo horns to gimbal arms
- The gimbal mounts at the very bottom of the rocket. The flex hose connects the fixed ball valve to the moving nozzle.

### Actuators Summary
| Servo | Purpose | Pin | Range |
|-------|---------|-----|-------|
| MG996R | Ball valve throttle | D9 | 0°-90° (closed-open) |
| MG90S #1 | TVC pitch | D5 | 78°-102° (±12° from center) |
| MG90S #2 | TVC yaw | D6 | 78°-102° (±12° from center) |

### Power
- **2S 7.4V 1000mAh LiPo** — powers everything
- **5V 3A BEC** — stable 5V for servos + Arduino + sensors
- No boost converter needed

### Data Logging
- **Micro SD card module** on SPI
- Log at 50Hz: timestamp, alt_baro, alt_tof, accel_xyz, gyro_xyz, pitch, roll, throttle_servo, tvc_pitch_servo, tvc_yaw_servo, state, co2_estimate

## Control Architecture

### State Machine
```
IDLE → ARMED → LAUNCH → ASCENT → HOVER → DESCENT → LANDED → SAFE
```

| State | Entry Condition | Throttle PID | TVC PID | Exit Condition |
|-------|----------------|-------------|---------|----------------|
| IDLE | Power on | OFF (valve closed) | OFF (centered) | Arm button press |
| ARMED | Button press | OFF | OFF (centered) | Second button press OR 30s timeout→IDLE |
| LAUNCH | Button from ARMED | Open to ~70% | Active (hold vertical) | Altitude > 0.5m |
| ASCENT | Alt > 0.5m | Velocity setpoint ~1.5 m/s | Active (hold vertical) | Altitude reaches target |
| HOVER | At target altitude | Altitude setpoint (hold) | Active (hold vertical) | Timer OR CO₂ < 15g |
| DESCENT | Hover timeout or low CO₂ | Velocity setpoint ~-0.5 m/s | Active (hold vertical) | Alt < 0.3m AND vel < 0.5 m/s |
| LANDED | Near ground + low vel | Valve closed | Centered | 3 seconds elapsed |
| SAFE | Post-landing | Valve closed | Centered | Power off |

### Dual PID System

**PID 1 — Altitude/Throttle Controller**
- Input: altitude error (target - measured) during HOVER, velocity error during ASCENT/DESCENT
- Output: ball valve servo angle (0°-90°)
- Runs at 50Hz
- Anti-windup on integrator (clamp ±20° contribution)
- Gains tuned in MATLAB first: Kp_alt, Ki_alt, Kd_alt

**PID 2 — Attitude/TVC Controller**
- Input: pitch error and roll error (target = 0° for both, i.e. vertical)
- Output: TVC gimbal servo angles for pitch and yaw (each ±12° from center)
- Runs at 50Hz (same loop as throttle PID)
- Two independent sub-controllers (one for pitch, one for roll)
- Gains tuned in MATLAB: Kp_att, Ki_att, Kd_att
- **Critical during hover** — this is the primary attitude stabilization. Fins help during ascent but are useless at hover velocity.

### Sensor Fusion
Complementary filter blending three altitude sources:
```
alt_fused = alpha * alt_baro + (1-alpha) * alt_accel_integrated
if (tilt < 20° AND tof_range < 4.0m):
    alt_fused = beta * alt_tof_corrected + (1-beta) * alt_fused
    where alt_tof_corrected = tof_distance * cos(tilt_angle)
```
- alpha ≈ 0.98 (barometer for drift-free baseline)
- beta ≈ 0.7 near ground (VL53L1X very precise at short range)

Attitude estimation from MPU6050:
```
pitch = atan2(accel_x, sqrt(accel_y² + accel_z²))
roll = atan2(accel_y, sqrt(accel_x² + accel_z²))
// Fuse with gyro for high-rate updates:
pitch_fused = 0.98 * (pitch_prev + gyro_x * dt) + 0.02 * pitch_accel
roll_fused = 0.98 * (roll_prev + gyro_y * dt) + 0.02 * roll_accel
```

### Safety Cutoffs (override any state)
- Tilt > 30° → close valve, center TVC, transition to SAFE
- Altitude > 10m → close valve (runaway protection)
- No altitude change for 5s during HOVER → close valve
- Battery voltage < 6.0V → close valve
- Any sensor failure (I2C timeout) → close valve
- CO₂ estimate depleted → transition to DESCENT

### CO₂ Remaining Estimator
- No direct measurement — estimate from cumulative flow
- co2_remaining -= valve_opening_fraction * max_flow_rate * dt
- max_flow_rate calibrated during static fire test (~10-13 g/s at full open, 850 psi, 2.5mm nozzle)
- When co2_remaining < 15g → trigger DESCENT
- Note: pressure decays as CO₂ cools during expansion. Algorithm should track estimated pressure drop and adjust flow rate model accordingly.

## Flight Physics (for reference in code)
- Liftoff mass: ~670-700g (no regulator = significant weight savings)
- Max thrust (full open, 850 psi, 2.5mm nozzle): ~15-20N
- Hover thrust: ~6.8N (= weight)
- Hover throttle: ~35-45% valve opening
- Total burn time (full open): ~8-10s
- Realistic hover duration at 5ft: 4-7 seconds
- TVC authority: ±12° gimbal → ~±3N lateral force at hover thrust

## Pin Assignments
| Pin | Function |
|-----|----------|
| GPIO 2 | Arm/launch push button (INPUT_PULLUP, latching switch) |
| GPIO 12 | Buzzer |
| GPIO 13 | Status LED |
| GPIO 26 | TVC pitch servo PWM |
| GPIO 33 | TVC yaw servo PWM |
| GPIO 25 | Throttle servo PWM (ball valve) |
| GPIO 47 | SD card CS |
| GPIO 10 | SD MOSI (HSPI) |
| GPIO 11 | SD MISO (HSPI) |
| GPIO 9 | SD SCK (HSPI) |
| GPIO 41 | I2C SDA (BMP388 + MPU6050 + VL53L1X) |
| GPIO 42 | I2C SCL |
| GPIO 1 | Battery voltage divider (ADC, with resistor divider 2S→3.3V range) |
| — | LoRa SX1262: on-board, uses internal SPI (GPIO 8/9/10 reserved by Heltec) |
| — | OLED SSD1306: on-board, I2C addr 0x3C (shares I2C bus with sensors) |

**Important Heltec V3 pin notes:**
- GPIO 8, 9, 10 are used internally by the SX1262 LoRa — do NOT assign to other functions
- GPIO 36, 37 are used by the OLED RST/display — do NOT assign
- Use HSPI (not VSPI) for the SD card to avoid conflicts with on-board LoRa SPI
- All servo pins must be on channels that support LEDC PWM (most GPIO work)
- 3.3V logic: servos need 5V power from BEC but accept 3.3V signal (MG996R and MG90S both work with 3.3V signal)

## Build & Test Philosophy
1. **Breadboard first** — all development on solderless breadboard
2. **Sensor drivers individually** — verify each sensor reads correctly
3. **MATLAB simulation** — tune all PID gains in simulation before hardware
4. **Bench test full firmware** — state machine by moving breadboard by hand
5. **Static fire** — propulsion clamped in vise, servo cycles, thrust measured
6. **TVC bench test** — gimbal range of motion, servo response, pushrod travel
7. **Tethered flight** — fishing line to stake, 5ft limit
8. **Free flight** — start with short hops, increment to full hover attempts

## Libraries (PlatformIO)
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
    arduino-libraries/SD
    madhephaestus/ESP32Servo
    sandeepmistry/LoRa
    thingpulse/ESP8266 and ESP32 OLED driver for SSD1306 displays
```

## File Structure
```
sparc/
├── firmware/
│   ├── CLAUDE.md              (this file)
│   ├── platformio.ini
│   ├── src/
│   │   └── main.cpp           (entry point: setup + loop)
│   └── lib/
│       ├── state_machine/     (flight state machine + transitions)
│       ├── sensors/           (BMP388, MPU6050, VL53L1X drivers)
│       ├── fusion/            (complementary filter for alt + attitude)
│       ├── pid/               (dual PID: throttle + TVC)
│       ├── actuator/          (throttle servo + TVC gimbal servos)
│       ├── logger/            (SD card CSV logging)
│       ├── telemetry/         (LoRa downlink)
│       └── safety/            (cutoff checks)
├── simulation/
│   ├── sparc_sim.m            (single-axis throttle sim)
│   ├── sparc_tvc_sim.m        (dual PID with TVC)
│   └── pid_tuner.m            (gain tuning helper)
├── cad/
│   ├── gimbal.step
│   ├── sled.step
│   └── stl/
├── docs/
│   ├── SPARC_Project_Documentation.md
│   ├── flight_logs/
│   ├── parts_list.xlsx
│   └── website/
├── README.md
├── .gitignore
└── LICENSE
```

## Key Design Decisions Log
1. Switched from compressed air/water to CO₂ for throttleability
2. Switched from 12g to 88g cartridge for sufficient burn time during hover
3. Removed pressure regulator — run full cartridge pressure for simplicity and weight savings (saves ~245g)
4. Upgraded from solenoid (binary) to servo + ball valve (proportional) for smooth throttle control
5. Added 2-axis TVC gimbal to solve the zero-velocity stability problem (fins useless during hover)
6. Added flexible PTFE hose between fixed valve and moving nozzle to enable TVC
7. Upgraded from Arduino Nano to Nano Every for more PWM pins and memory
8. Added LoRa telemetry for real-time flight monitoring
