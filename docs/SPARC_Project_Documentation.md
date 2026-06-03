# SPARC — Student Propulsive Autonomous Rocketry Challenge

## Project Overview

Self-contained CO₂-powered rocket with proportional thrust control and 2-axis Thrust Vector Control (TVC) for autonomous hover at ~5 ft and propulsive soft landing.

### Current Configuration

- **No pressure regulator** — full cartridge pressure (~850 psi) feeds directly into a 1000psi-rated stainless ball valve
- **Proportional throttle** — MG996R servo actuates the ball valve for 0-100% flow control
- **2-axis TVC** — 3D-printed gimbal with two MG90S servos tilts the nozzle ±12° via steel pushrods
- **Flexible PTFE hose** — braided stainless hose between fixed ball valve and moving nozzle enables gimbal articulation
- **Heltec WiFi LoRa 32 V3** — ESP32-S3 flight computer with built-in SX1262 LoRa (915MHz) + OLED display. Replaces Arduino Nano Every.
- **Dual PID** — one loop for altitude/throttle, one for attitude/TVC
- **Full sensor suite** — BMP388 altimeter + MPU6050 IMU + VL53L1X laser rangefinder + SD logging

### Key Design Decisions

| # | Decision | Rationale |
|---|----------|-----------|
| 1 | CO₂ over compressed air/water | Throttleable. Water dumps in 0.1s, CO₂ gives seconds of controllable burn. |
| 2 | 88g over 12g cartridge | 7× more propellant. 12g lasts <1s at useful thrust — no hover possible. |
| 3 | No pressure regulator | Saves 245g (was heaviest single component). All fittings rated ≥1000psi. |
| 4 | Servo + ball valve over solenoid | Proportional flow (0-100%) vs binary on/off. Critical for PID hover control. |
| 5 | 2-axis TVC over fin-only stabilization | Fins are useless at hover velocity (~0 m/s). TVC provides active attitude control. |
| 6 | Heltec ESP32 over Arduino Nano Every | Built-in LoRa + OLED + WiFi. More memory, faster clock, more PWM channels. |
| 7 | Flexible PTFE hose for TVC | Allows nozzle to gimbal while upstream plumbing stays fixed. |

---

## 1. Full Parts List

### Propulsion

| Item | Description | Qty | Est. Cost | Weight (g) | Source |
|------|-------------|-----|-----------|------------|--------|
| 88g CO₂ cartridges | Umarex 88g CO2 Capsules (2-pack) | 2 | $25.98 | 195 ea. | [Amazon](https://www.amazon.com/s?k=umarex+88g+co2) |
| Piercing adapter | MCS 88g/90g CO2 Adapter with On/Off Valve | 1 | $25.00 | 40 | [MCS](https://mcsus.com/products/88g-90g-3oz-disposable-co2-adapter-with-on-off-valve) |
| G1/2 → 1/4" NPT adapter | Metalwork 304 SS G1/2 Female to 1/4" NPT Male | 1 | $8.00 | 25 | [Amazon](https://www.amazon.com/Metalwork-Stainless-Fitting%EF%BC%8CConverter-Adapter-Female/dp/B0C3GDNWLB?th=1) |
| 1/4" → 1/8" NPT reducer | 304 SS 1/4" NPT Female to 1/8" NPT Male | 1 | $8.00 | 15 | [Amazon](https://www.amazon.com/Stainless-Reducing-Fittings-Extension-Connector/dp/B0FGXS7C3N) |
| O-rings (G1/2 seal) | Ninja CO2/HPA Urethane Tank O-Rings (10-pack) | 1 | $4.50 | <1 | [Amazon](https://www.amazon.com/Ninja-Co2-Urethane-Tank-Orings/dp/B00I5S9NCA) |
| Ball valve | ARITA 1/8" NPT Stainless 304 Mini Ball Valve, 1000 PSI | 1 | $14.00 | 30 | [Amazon](https://www.amazon.com/s?k=arita+1%2F8+npt+stainless+ball+valve) |
| Throttle servo | MG996R Metal Gear Servo (4-pack) | 1 | $13.99 | 55 | [Amazon](https://www.amazon.com/4-Pack-MG996R-Torque-Digital-Helicopter/dp/B07MFK266B) |
| TVC flexible hose | Earl's Speed-Flex 6" Braided PTFE Hose (1/8" NPT M-F) | 1 | $20.00 | 15 | [Real Street Performance](https://www.realstreetperformance.com/earls-speed-flex-1-8-npt-male-to-1-8-npt-female-stainless-steel-braided-ptfe-brake-hose-6.html) |
| TVC gimbal | 3D printed 2-axis gimbal (K-9 style or custom) | 1 | — | 20 | 3D print |
| TVC servos | MG90S Metal Gear Servos (2×) | 2 | $12.00 | 28 | [Amazon](https://www.amazon.com/s?k=mg90s) |
| TVC pushrods | Steel Servo Pushrods 1.2mm | 1 pack | $8.00 | 5 | [Amazon](https://www.amazon.com/Hobbypark-%CE%A61-2mm-L120mm-Airplane-Replacement/dp/B01EG3RQJE) |
| Nozzle | 1/8" NPT Brass Pipe Cap (drill 2.0-2.5mm orifice) | 2 | $9.98 | 12 | [Amazon](https://www.amazon.com/s?k=1%2F8+npt+brass+pipe+cap) |
| Thread sealant | Teflon Tape + Loctite 545 | 1 | $10.00 | 0 | Amazon |

**Propulsion subtotal:** ~$149 | ~440g wet

### Avionics (Heltec ESP32 as flight computer)

| Item | Description | Qty | Est. Cost | Weight (g) | Source |
|------|-------------|-----|-----------|------------|--------|
| Flight computer | Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + OLED) | 1 | $22.00 | 12 | [Amazon](https://www.amazon.com/s?k=heltec+wifi+lora+32+v3) |
| Barometric altimeter | BMP388 digital pressure sensor module | 1 | $8.99 | 1.5 | [Amazon](https://www.amazon.com/BMP388-Digital-Temperature-Atmospheric-Pressure/dp/B0DFY1FZ67) |
| 6-axis IMU | GY-521 MPU6050 accelerometer/gyroscope module | 1 | $7.49 | 3 | [Amazon](https://www.amazon.com/s?k=GY-521+MPU6050) |
| ToF rangefinder | TOF400C VL53L1X 4M laser distance sensor | 1 | $8.99 | 1 | [Amazon](https://www.amazon.com/s?k=TOF400C+VL53L1X) |
| SD card module | HiLetgo Micro SD TF Card Reader (5-pack) | 1 | $7.99 | 5 | [Amazon](https://www.amazon.com/s?k=HiLetgo+micro+SD+module+SPI) |
| Micro SD card | 16GB microSDHC | 1 | $7.99 | 1 | [Amazon](https://www.amazon.com/s?k=16GB+micro+SD+card) |

**Avionics subtotal:** ~$63 | ~24g

*Note: LoRa telemetry, OLED display, and WiFi are all built into the Heltec board — no separate modules needed.*

### Power

| Item | Description | Qty | Est. Cost | Weight (g) | Source |
|------|-------------|-----|-----------|------------|--------|
| Battery | 2S 7.4V 1000mAh 35C LiPo | 1 | $12.99 | 60 | [Amazon](https://www.amazon.com/s?k=2S+7.4V+1000mAh+LiPo) |
| Voltage regulator | FainWan 3A BEC 5V/6V adjustable (6-25.2V input) | 1 | $7.99 | 8 | [Amazon](https://www.amazon.com/s?k=FainWan+3A+BEC+5V+6V) |
| Arm/launch button | Gebildet 16mm SS Latching Push Button LED 12-24V | 1 | $10.98 | 10 | [Amazon](https://www.amazon.com/s?k=Gebildet+16mm+latching+push+button+LED) |
| Power switch | Mini SPDT slide switch | 1 | $5.49 | 1 | [Amazon](https://www.amazon.com/s?k=mini+slide+switch+SPDT) |

**Power subtotal:** ~$37 | ~79g

### Wiring & Prototyping

| Item | Description | Qty | Est. Cost | Source |
|------|-------------|-----|-----------|--------|
| Breadboards | Half-size solderless (3-pack) | 1 | $6.99 | Amazon |
| Jumper wires | Dupont M-M / M-F / F-F kit | 1 | $6.99 | Amazon |
| Proto boards | Mini PCB prototype board (10-pack) | 1 | $6.49 | Amazon |
| Wire | 22 AWG silicone wire red + black | 1 | $8.99 | Amazon |
| Heat shrink + zip ties | Assortment kit | 1 | $8.99 | Amazon |
| Soldering kit | Iron + solder + flux | 1 | $15.99 | Amazon |
| Multimeter | Digital multimeter | 1 | $12.99 | Amazon |

**Wiring subtotal:** ~$67

### Structure

| Item | Description | Qty | Est. Cost | Weight (g) | Source |
|------|-------------|-----|-----------|------------|--------|
| Body tube (test) | Cardboard rocket tube (~35-40" long) | 2 | — | 35-40 | You own |
| Body tube (final) | Clear polycarbonate tube 2" OD × 14-18" | 1 | $13.99 | 110 | Amazon |
| Fins | Clear polycarbonate sheet 1/8" × 6" × 12" | 1 | $8.99 | 25 | Amazon |
| Nose cone | 3D printed PETG | 1 | — | 20 | 3D print |
| Internal mounts | 3D printed sled, bulkheads, servo mount, gimbal cradle | 1 set | — | 30 | 3D print |
| Hardware | M3 standoffs, bolts, nuts, nylon spacers | 1 kit | $8.99 | 15 | Amazon |
| Adhesive | Clear epoxy or Weld-On 16 | 1 | $12.99 | 0 | Amazon |

**Structure subtotal:** ~$45 | ~235g (cardboard test) / ~310g (polycarb final)

### Safety

| Item | Description | Qty | Est. Cost | Source |
|------|-------------|-----|-----------|--------|
| Safety glasses | ANSI Z87.1 (3-pack) | 1 | $9.99 | Amazon |
| Work gloves | Mechanic gloves (CO₂ cold protection) | 1 | $12.99 | Amazon |
| Leak testing | Spray bottle + dish soap | 1 | — | You own |

**Safety subtotal:** ~$23

### Totals

| Configuration | Wet liftoff mass | Estimated cost |
|--------------|-----------------|----------------|
| Cardboard test tube | ~780-820g | ~$380-420 |
| Clear polycarb final | ~855-895g | ~$400-440 |

---

## 2. Plumbing Chain

```
88g CO₂ Cartridge (male M16×1.5)
    ↓ screws in, piercing pin punctures brass seal
MCS Piercing Adapter with On/Off Valve (M16×1.5 female → G1/2 male)
    ↓ + polyurethane o-ring on G1/2 joint
Metalwork 304 SS Adapter (G1/2 female → 1/4" NPT male)
    ↓
304 SS Reducer (1/4" NPT female → 1/8" NPT male)
    ↓
ARITA 1/8" NPT SS Ball Valve — 1000 PSI rated (throttle, driven by MG996R servo)
    ↓
Earl's Speed-Flex 6" Braided PTFE Hose (1/8" NPT M-F, flexes for gimbal)
    ↓
Brass Pipe Cap Nozzle (2.0-2.5mm drilled orifice, mounted in TVC gimbal)
```

**Pressure throughout entire chain: ~850 psi (no regulator).** All components rated ≥1000 psi.

**Thread compatibility verified at every joint:**

| Joint | Male thread | Female thread | Seal method |
|-------|------------|---------------|-------------|
| Cartridge → MCS adapter | M16×1.5 (cartridge) | M16×1.5 (adapter) | O-ring + piercing pin |
| MCS adapter → Metalwork | G1/2 male (MCS) | G1/2 female (Metalwork) | Polyurethane o-ring |
| Metalwork → Reducer | 1/4" NPT male | 1/4" NPT female | Teflon tape |
| Reducer → Ball valve | 1/8" NPT male | 1/8" NPT female (ARITA) | Teflon tape |
| Ball valve → Hose | 1/8" NPT (valve out) | 1/8" NPT (hose) | Teflon tape |
| Hose → Nozzle | 1/8" NPT (hose out) | 1/8" NPT (cap) | Teflon tape |

---

## 3. Flight Computer

### Heltec WiFi LoRa 32 V3

| Feature | Spec |
|---------|------|
| MCU | ESP32-S3, 240 MHz dual-core |
| Flash | 8MB |
| SRAM | 320KB |
| LoRa | SX1262, 915MHz (built-in) |
| Display | 0.96" OLED SSD1306 (built-in) |
| WiFi | 802.11 b/g/n (built-in) |
| Bluetooth | BLE 5.0 (built-in) |
| Logic | 3.3V (all sensors compatible) |

### Pin Assignments

| GPIO | Function |
|------|----------|
| 2 | Arm/launch push button (latching, INPUT_PULLUP) |
| 19 | Buzzer |
| 20 | Status LED |
| 47 | Throttle servo PWM (MG996R → ball valve) |
| 48 | TVC pitch servo PWM (MG90S #1) |
| 3 | TVC yaw servo PWM (MG90S #2) |
| 41 | I2C SDA (BMP388 + MPU6050 + VL53L1X) |
| 42 | I2C SCL |
| 1 | Battery voltage divider (ADC) |
| 26 | Spare (available; freed because LoRa is unused) |
| — | OLED SSD1306: on-board (SDA_OLED/SCL_OLED/RST_OLED, Heltec BSP); I2C 0x3C on Wire1 |
| — | Telemetry: WiFi SoftAP + UDP — no SD card, LoRa radio left disabled |

**Reserved pins (Heltec V3 — do not use for external hardware):**
- GPIO 6, 7: ESP32-S3FN8 internal SPI flash — reassigning bricks the MCU
- GPIO 8–14: on-board SX1262 LoRa SPI (not broken out)
- GPIO 26–32: SPI-flash interface (26 usable as a spare only; 27–32 off-limits)
- GPIO 0, 45, 46: strapping/boot pins
- SDA_OLED / SCL_OLED / RST_OLED: on-board OLED pins (Heltec BSP)

### Servo Summary

| Servo | Purpose | Pin | Range | Torque |
|-------|---------|-----|-------|--------|
| MG996R | Ball valve throttle | GPIO 47 | 0°-90° | 12 kg·cm @ 6V |
| MG90S #1 | TVC pitch | GPIO 48 | 78°-102° (±12°) | 2 kg·cm |
| MG90S #2 | TVC yaw | GPIO 3 | 78°-102° (±12°) | 2 kg·cm |

---

## 4. Control Architecture

### State Machine

```
IDLE → ARMED → LAUNCH → ASCENT → HOVER → DESCENT → LANDED → SAFE
```

| State | Entry | Throttle | TVC | Exit |
|-------|-------|----------|-----|------|
| IDLE | Power on | Closed | Centered | Arm button press |
| ARMED | Button press | Closed | Centered | 2nd press or 30s timeout→IDLE |
| LAUNCH | Button from ARMED | ~70% open | Active | Alt > 0.5m |
| ASCENT | Alt > 0.5m | Vel setpoint 1.5 m/s | Active | Alt reaches target |
| HOVER | At target | Alt setpoint (hold) | Active | Timer or CO₂ < 15g |
| DESCENT | Hover end | Vel setpoint -0.5 m/s | Active | Alt < 0.3m, vel < 0.5 m/s |
| LANDED | Near ground | Closed | Centered | 3s elapsed |
| SAFE | Post-landing | Closed | Centered | Power off |

### Dual PID System

**PID 1 — Altitude/Throttle**
- Input: altitude error (HOVER) or velocity error (ASCENT/DESCENT)
- Output: ball valve servo angle (0°-90°)
- Rate: 50Hz
- Gains: Kp_alt, Ki_alt, Kd_alt (tuned in MATLAB)

**PID 2 — Attitude/TVC**
- Input: pitch error, roll error (target = 0° for both)
- Output: gimbal servo angles (±12° from center)
- Rate: 50Hz
- Two independent sub-controllers (pitch, roll)
- Gains: Kp_att, Ki_att, Kd_att (tuned in MATLAB)

### Sensor Fusion

**Altitude (complementary filter):**
```
alt_fused = 0.98 * alt_baro + 0.02 * alt_accel_integrated
if tilt < 20° AND tof_range < 4.0m:
    alt_tof_corrected = tof_distance * cos(tilt_angle)
    alt_fused = 0.7 * alt_tof_corrected + 0.3 * alt_fused
```

**Attitude (gyro + accel fusion):**
```
pitch = 0.98 * (pitch_prev + gyro_x * dt) + 0.02 * atan2(accel_x, sqrt(accel_y² + accel_z²))
roll  = 0.98 * (roll_prev  + gyro_y * dt) + 0.02 * atan2(accel_y, sqrt(accel_x² + accel_z²))
```

### Safety Cutoffs (override any state)

| Condition | Action |
|-----------|--------|
| Tilt > 30° | Close valve, center TVC → SAFE |
| Altitude > 10m | Close valve (runaway protection) |
| No alt change for 5s in HOVER | Close valve → SAFE |
| Battery < 6.0V | Close valve → SAFE |
| I2C sensor timeout | Close valve → SAFE |
| CO₂ estimate depleted | Transition → DESCENT |

---

## 5. Flight Physics

| Parameter | Value |
|-----------|-------|
| Liftoff mass (cardboard test) | ~800g |
| Max thrust (full open, 850 psi, 2.5mm nozzle) | 15-20N |
| Hover thrust (= weight) | ~7.8N |
| Hover throttle | ~35-50% valve opening |
| TWR at launch | ~2.0-2.5 |
| Total burn (full open) | ~8-10s |
| Realistic hover at 5ft | 4-7 seconds |
| Total powered flight | 8-12 seconds |
| TVC authority | ±12° gimbal → ~±3N lateral force |
| CO₂ per flight | ~70-80g of 88g |
| Cost per flight | ~$7 (one cartridge) |

### Flight Profile

| Phase | Duration | Thrust | Flow rate | CO₂ used |
|-------|----------|--------|-----------|----------|
| Ascent to 5ft | 2-3s | 12-15N | 10-13 g/s | ~25g |
| Hover at 5ft | 4-6s | ~7.8N | 5-7 g/s | ~30g |
| Controlled descent | 2-3s | 4-6N | 4-6 g/s | ~15g |
| Landing | <2 mph | 0N | 0 | 0 |

---

## 6. Build & Testing Procedures

### Phase 1: Sensor Validation (breadboard)
1. Wire BMP388 + MPU6050 + VL53L1X to Heltec ESP32 on breadboard (I2C: SDA=GPIO 41, SCL=GPIO 42)
2. Verify each sensor individually on Serial Monitor
3. Confirm I2C addresses: BMP388=0x76, MPU6050=0x68, VL53L1X=0x29, OLED=0x3C
4. Bring up WiFi telemetry (SoftAP "SPARC-Telemetry"); join from a laptop
5. Verify the 50Hz UDP JSON stream (`nc -ul 4210`)

### Phase 2: MATLAB Simulation
1. Run `sparc_tvc_sim.m` in MATLAB Online
2. Tune Kp_alt, Ki_alt, Kd_alt for stable hover (altitude plot should be flat)
3. Tune Kp_att, Ki_att, Kd_att for stable attitude (tilt should stay < 5°)
4. Add sensor noise, verify controller still works
5. Record final gains → paste into firmware constants

### Phase 3: Servo & Actuator Testing (breadboard)
1. Connect MG996R to GPIO 47, verify 0-90° sweep
2. Connect MG90S ×2 to GPIO 48/3, verify ±12° from center (78°-102°)
3. Wire 2S LiPo → BEC → breadboard, verify 5V stable under 3A peak servo load
4. Test ball valve actuation: servo 0°=closed, 90°=open, verify smooth motion via linkage
5. Test gimbal articulation: both axes sweep smoothly, pushrods don't bind

### Phase 4: Propulsion Bench Testing (CRITICAL SAFETY)
1. Assemble full plumbing chain with nozzle capped (temporary plug)
2. Wear safety glasses. Point nozzle away from all people.
3. Open MCS on/off valve, screw in 88g cartridge (piercing adapter punctures seal)
4. Soapy water spray on EVERY joint — bubbles = leak
5. Fix all leaks before proceeding. If hissing: close MCS valve, vent gas, fix, retry.
6. Static fire: clamp assembly in vise, remove nozzle cap, connect servo to bench power
7. Cycle valve open/closed. Measure thrust with kitchen scale if possible.
8. Record: thrust at full open, burn duration, gas temperature behavior

### Phase 5: TVC Bench Testing
1. Mount gimbal + nozzle on test stand (NOT in rocket yet)
2. Connect both MG90S servos + pushrods
3. Command full range of motion (±12° pitch, ±12° yaw)
4. Verify no binding, smooth response, pushrods stay connected
5. Fire propulsion with TVC active: command deflections during gas flow, verify gimbal holds position against thrust

### Phase 6: Full Software Integration (breadboard)
1. Implement complete state machine (IDLE → SAFE)
2. Integrate dual PID with MATLAB-tuned gains
3. Test state transitions by hand-moving breadboard
4. Verify WiFi telemetry streams all fields at 50Hz (recorded ground-side)
5. Test the downlink: a laptop on the SoftAP receives the real-time UDP stream
6. Verify all safety cutoffs trigger correctly

### Phase 7: Rocket Integration
1. Solder breadboard circuit → proto board
2. Build electronics sled + propulsion cradle (3D printed)
3. Mount all hardware in cardboard test tube
4. Wire everything, verify sensors read correctly with tube sealed
5. Weight check: verify liftoff mass matches predictions

### Phase 8: Flight Testing
1. **Tethered flight**: fishing line to stake, 5ft max. Verify launch detect, state transitions, TVC actuation. Review the recorded telemetry stream.
2. **Low-pressure hops**: short bursts, verify stability. Review data after EVERY attempt.
3. **Full hover**: incrementally increase hover duration. Target: stable 5ft hold for 4+ seconds.
4. **Always have recovery ready**: streamer/parachute for early flights before hover is proven.

---

## 7. MATLAB Simulator

### Dual PID + TVC Simulation

```matlab
%% SPARC Flight Simulator — Dual PID (Throttle + TVC)
clear; clc; close all;

%% === TUNE THESE ===
Kp_alt = 18;  Ki_alt = 8;   Kd_alt = 25;    % Altitude/throttle PID
Kp_att = 22;  Ki_att = 12;  Kd_att = 18;    % Attitude/TVC PID
max_gimbal_deg = 12;                          % ±12° TVC range

%% === ROCKET PARAMETERS ===
dry_mass    = 0.670;       % kg (everything except CO2)
co2_initial = 0.088;       % kg (88g cartridge)
reg_psi     = 850;         % NO REGULATOR — full cartridge pressure
nozzle_d    = 0.0025;      % 2.5mm orifice
target_alt  = 1.524;       % 5 ft in meters
descent_rate = -0.5;       % target descent m/s

%% === PHYSICS ===
g = 9.81; dt = 0.001; t_end = 20;
P = reg_psi * 6894.76;
A_noz = pi * (nozzle_d/2)^2;
R_co2 = 188.9; T_gas = 293; gamma = 1.3;
Cd_drag = 0.5; A_body = pi*0.025^2; rho_air = 1.18;

rho_gas = P / (R_co2 * T_gas);
v_exit = sqrt(2 * P / rho_gas);
mdot_max = rho_gas * A_noz * v_exit;
F_max = mdot_max * v_exit + P * A_noz;

fprintf('Max thrust: %.1f N\n', F_max);
fprintf('Max mdot:   %.1f g/s\n', mdot_max*1000);
fprintf('Hover throttle: %.0f%%\n', (dry_mass+co2_initial/2)*g/F_max*100);

%% === SIMULATION ===
N = round(t_end/dt);
t=zeros(N,1); alt=zeros(N,1); vel=zeros(N,1);
pitch_log=zeros(N,1); thrust_log=zeros(N,1);
co2_log=zeros(N,1); tvc_log=zeros(N,1);

co2=co2_initial; v=0; h=0; pitch=0; pitch_rate=0;
int_alt=0; int_att=0; prev_err_alt=0; prev_err_att=0;
state=1; % 1=ascent,2=hover,3=descent,4=landed

for i=1:N
    t(i)=(i-1)*dt;
    m=dry_mass+co2; weight=m*g;

    % State transitions
    if state==1 && h>=target_alt*0.95, state=2; int_alt=0; end
    if state==2 && co2<0.015, state=3; int_alt=0; end
    if state==3 && h<=0.05 && v>-0.3, state=4; end

    % Altitude PID
    if state==1, sp=1.5; meas=v;
    elseif state==2, sp=target_alt; meas=h;
    elseif state==3, sp=descent_rate; meas=v;
    else, sp=0; meas=0; end

    err_alt=sp-meas;
    int_alt=max(-5,min(5,int_alt+err_alt*dt));
    d_alt=(err_alt-prev_err_alt)/dt; prev_err_alt=err_alt;

    if state<=3
        valve_cmd=Kp_alt*err_alt+Ki_alt*int_alt+Kd_alt*d_alt;
        valve_pct=max(0,min(100,valve_cmd));
    else, valve_pct=0; end

    % Attitude PID (TVC)
    err_att=0-pitch;
    int_att=max(-5,min(5,int_att+err_att*dt));
    d_att=(err_att-prev_err_att)/dt; prev_err_att=err_att;
    tvc_cmd=Kp_att*err_att+Ki_att*int_att+Kd_att*d_att;
    tvc_deg=max(-max_gimbal_deg,min(max_gimbal_deg,tvc_cmd));

    % Thrust
    valve_frac=sin(valve_pct/100*pi/2);
    if co2>0.001
        T_curr=T_gas-2*(co2_initial-co2)/co2_initial*30;
        T_curr=max(T_curr,253);
        pf=T_curr/T_gas;
        F_thrust=F_max*valve_frac*pf;
        mdot_now=mdot_max*valve_frac*pf;
        co2=co2-mdot_now*dt;
        if co2<0, co2=0; F_thrust=0; end
    else, F_thrust=0; end

    % TVC lateral force
    F_lateral=F_thrust*sind(tvc_deg);
    F_vertical=F_thrust*cosd(tvc_deg);

    % Pitch dynamics (simplified)
    torque=F_lateral*0.3; % 0.3m moment arm (nozzle to CG)
    I_rocket=m*0.15^2; % approximate MOI
    pitch_accel=torque/I_rocket;
    pitch_rate=pitch_rate+pitch_accel*dt;
    pitch=pitch+pitch_rate*dt;

    % Add random disturbance torque (wind, asymmetry)
    pitch_rate=pitch_rate+(randn*0.5)*dt;

    % Vertical dynamics
    drag=0.5*Cd_drag*rho_air*A_body*v*abs(v);
    F_net=F_vertical-weight-drag*sign(v);
    a=F_net/m;
    v=v+a*dt; h=h+v*dt;
    if h<0 && i>100, h=0; v=0; state=4; end

    alt(i)=h; vel(i)=v; pitch_log(i)=pitch;
    thrust_log(i)=F_thrust; co2_log(i)=co2*1000;
    tvc_log(i)=tvc_deg;
end

%% === PLOTS ===
figure('Position',[100 100 900 900]);

subplot(5,1,1);
plot(t,alt*3.281,'Color',[0.1 0.6 0.4],'LineWidth',1.5);
hold on; yline(target_alt*3.281,'--r','Target');
ylabel('Alt (ft)'); title('SPARC Dual PID + TVC Simulation'); grid on;

subplot(5,1,2);
plot(t,vel*3.281,'Color',[0.85 0.35 0.15],'LineWidth',1.5);
ylabel('Vel (ft/s)'); yline(0,'--k'); grid on;

subplot(5,1,3);
plot(t,thrust_log,'Color',[0.2 0.5 0.85],'LineWidth',1.5);
hold on; yline((dry_mass+co2_initial/2)*g,'--r','Hover thrust');
ylabel('Thrust (N)'); grid on;

subplot(5,1,4);
plot(t,pitch_log,'Color',[0.6 0.2 0.7],'LineWidth',1.5);
ylabel('Pitch (deg)'); yline(0,'--k'); grid on;

subplot(5,1,5);
plot(t,tvc_log,'Color',[0.8 0.5 0.1],'LineWidth',1.5);
ylabel('TVC (deg)'); xlabel('Time (s)'); grid on;

% Summary
[~,idx]=max(alt);
land_idx=find(alt(100:end)<0.01,1)+99;
if isempty(land_idx), land_idx=N; end
fprintf('\n=== FLIGHT SUMMARY ===\n');
fprintf('Peak altitude:  %.1f ft\n', max(alt)*3.281);
fprintf('Total flight:   %.1f s\n', t(land_idx));
fprintf('CO2 used:       %.1f g\n', co2_initial*1000-co2_log(land_idx));
fprintf('Max tilt:       %.1f deg\n', max(abs(pitch_log)));
fprintf('Landing speed:  %.1f mph\n', abs(vel(max(1,land_idx-1)))*2.237);
fprintf('Gains: Kp_alt=%.0f Ki_alt=%.0f Kd_alt=%.0f\n', Kp_alt,Ki_alt,Kd_alt);
fprintf('       Kp_att=%.0f Ki_att=%.0f Kd_att=%.0f\n', Kp_att,Ki_att,Kd_att);
```

---

## 8. Repository Structure

```
sparc/
├── firmware/                  ← PlatformIO project (Heltec ESP32)
│   ├── CLAUDE.md              ← Full flight controller spec for Claude Code
│   ├── platformio.ini         ← Build config + library deps
│   ├── src/main.cpp           ← Entry point
│   └── lib/                   ← Modular libraries
│       ├── state_machine/
│       ├── sensors/
│       ├── fusion/
│       ├── pid/
│       ├── actuator/
│       ├── logger/
│       ├── telemetry/
│       └── safety/
├── simulation/                ← MATLAB scripts
│   ├── sparc_sim.m            ← Single-axis throttle sim
│   ├── sparc_tvc_sim.m        ← Dual PID + TVC sim
│   └── pid_tuner.m
├── cad/                       ← Fusion 360 models
│   ├── gimbal.step
│   ├── sled.step
│   └── stl/
├── docs/
│   ├── SPARC_Project_Documentation.md  ← This file
│   ├── flight_logs/
│   └── website/
├── README.md
├── .gitignore
└── LICENSE
```

---

## 9. Suppliers

| Supplier | Items | URL |
|----------|-------|-----|
| Amazon | Everything except MCS adapter | amazon.com |
| MCS (MCS US) | 88g piercing adapter with on/off valve | mcsus.com |
| Real Street Performance | Earl's braided PTFE flex hose | realstreetperformance.com |

3 suppliers total.
