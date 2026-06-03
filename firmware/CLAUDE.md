# SPARC Flight Controller

## Project Overview
SPARC (Student Propulsive Autonomous Rocketry Challenge) is a CO₂-powered rocket with proportional thrust control AND 2-axis Thrust Vector Control (TVC) for autonomous hover at ~5 feet and propulsive soft landing. The rocket maintains vertical orientation actively via TVC gimbal — not passively via fins alone.

## Architecture Summary
- **No pressure regulator** — full cartridge pressure (~850 psi) feeds directly into a 1000psi-rated ball valve
- **Proportional throttle** — MG996R servo actuates a stainless ball valve for 0-100% flow control
- **2-axis TVC** — Nozzle mounted in a 3D-printed gimbal, driven by two MG90S servos via pushrods, connected to the valve by a flexible braided PTFE hose
- **Dual PID** — One loop for altitude/throttle, one for attitude/TVC
- **Full sensor suite** — BMP388 + MPU6050 + VL53L1X + WiFi telemetry/logging

## Hardware

### Microcontroller
- **Heltec WiFi LoRa 32 V3** (ESP32-S3FN8, 240 MHz dual-core, 8MB flash, 320KB SRAM)
- Built-in WiFi — used as a SoftAP for live telemetry/logging to a ground-station laptop
- Built-in 0.96" OLED display — pre-flight status, sensor checks, state display on pad
- Built-in SX1262 LoRa radio (915MHz) is present but **not used** — enabling it locks
  GPIO 19/20/26 (its internal SPI), which are needed for buzzer/LED/spare
- 3.3V logic — all sensors are compatible
- Abundant GPIO + hardware PWM channels for 3 servos
- Use ESP32Servo library (not arduino Servo)

### Sensors (all on I2C bus)
- **BMP388** (addr 0x76 or 0x77): Barometric altitude, ±0.5m relative accuracy, up to 200Hz ODR
- **GY-521 MPU6050** (addr 0x68): 6-axis IMU (accel ±8g, gyro ±2000°/s). Primary attitude source for TVC.
- **TOF400C VL53L1X** (addr 0x29): Time-of-flight laser rangefinder, ±1mm accuracy to 4m, 50Hz max. Mount facing downward.

### Telemetry (built into flight computer)
- **WiFi SoftAP + UDP** (built-in ESP32-S3 radio): Real-time downlink of altitude, attitude, state, servo positions during flight. The board hosts AP "SPARC-Telemetry"; a ground-station laptop joins and receives the JSON stream (`nc -ul 4210`). No SD card — all flight data streams over WiFi, so there is no card to pull after landing.
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
| MG996R | Ball valve throttle | GPIO 47 | 0°-90° (closed-open) |
| MG90S #1 | TVC pitch | GPIO 48 | 78°-102° (±12° from center) |
| MG90S #2 | TVC yaw | GPIO 3 | 78°-102° (±12° from center) |

### Power
- **2S 7.4V 1000mAh LiPo** — powers everything
- **5V 3A BEC** — stable 5V for servos + Arduino + sensors
- No boost converter needed

### Data Logging
- **No SD card** — there are not enough usable GPIOs on the Heltec V3 for SD (4 SPI pins) plus 3 servos + button + buzzer + battery ADC. All logging is done over WiFi instead.
- Stream at 50Hz over WiFi/UDP: timestamp, alt_baro, alt_tof, accel_xyz, gyro_xyz, pitch, roll, throttle_servo, tvc_pitch_servo, tvc_yaw_servo, state, co2_estimate. A ground-station laptop records the stream to disk.

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
All pins below are physically broken out on the Heltec V3 headers and safe
to use. The sensors share one I2C bus on GPIO 41 (SDA) / 42 (SCL).

| Pin | Function |
|-----|----------|
| GPIO 41 | I2C SDA (BMP388 + MPU6050 + VL53L1X) |
| GPIO 42 | I2C SCL (BMP388 + MPU6050 + VL53L1X) |
| GPIO 47 | Throttle servo PWM (ball valve, MG996R) |
| GPIO 48 | TVC pitch servo PWM (MG90S #1) |
| GPIO 3 | TVC yaw servo PWM (MG90S #2) |
| GPIO 2 | Arm/launch push button (INPUT_PULLUP, latching switch) |
| GPIO 19 | Buzzer |
| GPIO 20 | Status LED |
| GPIO 1 | Battery voltage divider (ADC, with resistor divider 2S→3.3V range) |
| GPIO 26 | Spare (available; freed because LoRa is unused) |
| — | OLED SSD1306: on-board (SDA_OLED, SCL_OLED, RST_OLED — defined by the Heltec BSP); I2C addr 0x3C on Wire1 |

**Important Heltec V3 (ESP32-S3FN8) pin notes:**
- GPIO 6, 7 are the module's SPI flash — reassigning them crashes the MCU. NEVER use.
- GPIO 26–32 are the SPI-flash interface (26 = SPICS1, usable as a spare only because this module has no PSRAM). Treat 27–32 as off-limits.
- GPIO 8, 9, 10, 11, 12, 13, 14 are the on-board SX1262 LoRa SPI — not broken out, not used.
- GPIO 0, 45, 46 are strapping/boot pins — avoid for external hardware.
- OLED uses the BSP's SDA_OLED / SCL_OLED / RST_OLED macros — do NOT assign those GPIOs to anything else.
- All servo pins must be on channels that support LEDC PWM (most GPIO work).
- 3.3V logic: servos need 5V power from BEC but accept 3.3V signal (MG996R and MG90S both work with 3.3V signal).

## Build & Test Philosophy
1. **Breadboard first** — all development on solderless breadboard
2. **Sensor drivers individually** — verify each sensor reads correctly
3. **MATLAB simulation** — tune all PID gains in simulation before hardware
4. **Bench test full firmware** — state machine by moving breadboard by hand
5. **Static fire** — propulsion clamped in vise, servo cycles, thrust measured
6. **TVC bench test** — gimbal range of motion, servo response, pushrod travel
7. **Tethered flight** — fishing line to stake, 5ft limit
8. **Free flight** — start with short hops, increment to full hover attempts

## Commit & Push Guidelines

Commits follow the [Conventional Commits](https://www.conventionalcommits.org)
standard so history stays machine-readable and consistent.

**Subject line:** `type(scope): summary`
- **type** — one of: `feat` (new capability), `fix` (bug fix), `refactor`
  (behavior-preserving restructure), `perf`, `docs`, `test`, `build` (PlatformIO/
  toolchain/deps), `chore` (housekeeping).
- **scope** — the module touched, matching the `lib/` layout: `sensors`, `fusion`,
  `display`, `wifi_link`, `pid`, `tvc`, `state`, `safety`, or `main`. Omit if the
  change is genuinely cross-cutting.
- **summary** — imperative mood, lower-case, no trailing period, ≤ 72 chars
  ("add VL53L1X tilt correction", not "added" or "adds").

**Body (required when the change is more than a one-liner):**
- Describe the change against **the entire staged diff** — every file and concern
  it touches, not just the headline. If one commit spans multiple modules, cover
  each. Run `git diff --staged` and summarize what actually changed before writing.
- Explain the **why**, not just the what — the reasoning, the tradeoff, the bug's
  root cause. Wrap at ~72 chars. Use `-` bullets for distinct changes.
- Note hardware-affecting changes explicitly (pin reassignments, I2C bus, power
  rails) since they gate bring-up.

**Footer:** reference issues (`Refs #12`) and breaking changes (`BREAKING CHANGE:`)
when applicable.

**Process:**
- Commit and push only when explicitly asked.
- Never commit directly to `main` — branch first (`feature/…`, `fix/…`).
- Confirm the tree builds (`pio run`) before committing firmware changes.
- One logical change per commit; don't bundle unrelated work.

Example:
```
feat(fusion): add complementary-filter attitude estimator

The accel-only tilt was corrupted by thrust during powered flight and
gave only an unsigned tilt magnitude, unusable for the two-axis TVC PID.

- add lib/fusion/attitude with gyro+accel complementary filter (ALPHA 0.98),
  outputting signed pitch/roll seeded from the accelerometer
- main: compute loop dt, run the filter on fresh IMU samples, drop the old
  pitchDeg(), tilt-correct the ToF range with cos(pitch)·cos(roll)
- serial telemetry now reports pitch and roll
```

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
    madhephaestus/ESP32Servo
    thingpulse/ESP8266 and ESP32 OLED driver for SSD1306 displays
```
WiFi is built into the ESP32 Arduino framework — no library needed. SD and
LoRa libraries were removed (no SD card; LoRa radio left disabled).

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
│       ├── display/           (on-board OLED status screens)
│       ├── wifi_link/         (WiFi SoftAP + UDP telemetry/logging)
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
9. Moved to the Heltec WiFi LoRa 32 V3 (ESP32-S3) as the flight computer
10. Dropped the SD card — freed 4 SPI pins; flight data now streams over WiFi
11. Left the LoRa radio disabled and moved telemetry to a WiFi SoftAP, freeing GPIO 19/20/26 for buzzer/LED/spare
