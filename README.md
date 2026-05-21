# SPARC

**Student Propulsive Autonomous Rocketry Challenge**

A CO₂-powered rocket with proportional thrust control and 2-axis thrust vector control (TVC) for autonomous hover at 5 feet and propulsive soft landing.

## What makes this different

Most model rockets are ballistic — light the motor, hope for the best. SPARC actively controls its flight in real time:

- **Proportional throttle** — a servo-actuated ball valve meters CO₂ flow for smooth thrust modulation
- **Thrust vector control** — a 2-axis gimbal tilts the nozzle ±12° for active attitude stabilization during hover
- **Sensor fusion** — three altitude sources (barometric, IMU, laser rangefinder) fused for robust state estimation
- **Dual PID control** — one loop holds altitude, one keeps the rocket vertical
- **Autonomous state machine** — arms, launches, ascends, hovers, descends, and lands without human input

## Project Structure

```
sparc/
├── firmware/          Arduino flight controller (PlatformIO)
├── simulation/        MATLAB flight dynamics + PID tuning
├── cad/               Fusion 360 models (gimbal, sled, bulkheads)
├── docs/              Documentation, flight logs, parts list
└── README.md
```

## Hardware

| Subsystem | Key Component |
|-----------|--------------|
| Propulsion | 88g CO₂ cartridge, no regulator, full pressure (~850 psi) |
| Throttle | ARITA 1000psi stainless ball valve + MG996R servo |
| TVC | 3D-printed 2-axis gimbal + 2× MG90S servos + braided PTFE flex hose |
| Sensors | BMP388 + MPU6050 + VL53L1X |
| Computer | Arduino Nano Every |
| Telemetry | 915MHz LoRa |
| Power | 2S 7.4V LiPo + 5V BEC |

## Development

See [`firmware/CLAUDE.md`](firmware/CLAUDE.md) for the complete flight controller specification.

### Prerequisites

- [VS Code](https://code.visualstudio.com/) + [PlatformIO](https://platformio.org/)
- [MATLAB](https://matlab.mathworks.com/) (free online tier works)
- [Fusion 360](https://www.autodesk.com/products/fusion-360/) (free for students)

### Quick start

```bash
git clone https://github.com/YOUR_USERNAME/sparc.git
cd sparc/firmware
# PlatformIO auto-installs dependencies
# Connect Arduino Nano Every via USB
# Upload: pio run -t upload
# Monitor: pio device monitor -b 115200
```

## Flight Profile

| Phase | Duration | Thrust | Control |
|-------|----------|--------|---------|
| Ascent | 2-3s | 12-15N | Throttle PID + TVC active |
| Hover | 4-7s | ~6.8N (= weight) | Both PIDs holding altitude + vertical |
| Descent | 2-3s | 4-6N | Throttle PID targeting -0.5 m/s |
| Landing | — | 0N | Valve closed, < 2 mph touchdown |

## Safety

- Tilt > 30° → automatic abort (valve closes)
- All pressurized testing requires ANSI Z87.1 safety glasses
- CO₂ reaches -78°C during expansion — wear gloves
- Always test tethered before free flight
- Full safety procedures in [`docs/SPARC_Project_Documentation.md`](docs/SPARC_Project_Documentation.md)

## License

MIT — educational project. 
