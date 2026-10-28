# Firmware (ESP32 DevKit V1 / ESP32-WROOM-32, Arduino framework via PlatformIO)

## Layout
| Path | Content |
|---|---|
| `include/config.h` | **every tunable parameter** (X-macro table with units and descriptions) |
| `include/robot_types.h` | HalInputs/HalOutputs, directions, states, faults |
| `src/main.cpp` | ESP32 entry: 1 kHz loop, watchdog, NVS save, debug telemetry |
| `src/hal_esp32/` | pins, ADC, MPU6050 gyro (I2C), optional slot-encoder interrupts, LEDC PWM for L298N/TB6612/DRV8833, NVS (the only hardware-specific code) |
| `src/app/robot.*` | competition state machine, navigation, recovery, restarts |
| `src/sensors/line_sensor.*` | calibration, normalisation, line position, branch/goal features |
| `src/encoders/odometry.*` | distance and heading: quadrature encoders, slot encoders, or (no encoders) a motor speed model; turn angle from the gyro when fitted |
| `src/motion/motion_controller.*` | line-follow PD with speed profiles, straight, pivot, wheel PI (quadrature encoders) or feed-forward + learned left/right motor trim (slot encoders or none) |
| `src/maze/` | junction detector, graph |
| `src/goal_detection/` | end-box detector |
| `src/exploration/` | explorer policies, relocaliser |
| `src/planner/` | Dijkstra, frontier search, lower-bound certificate |
| `src/persistence/` | CRC-protected record of stage, calibration, map and route |
| `src/safety/` | stall and battery supervision (no-encoder stall check lives in `app/robot.cpp`) |
| `src/util/` | CRC32, ring-buffer logger |
| `src/sim_api/` | C ABI for the PC simulator (compiled only when `MESH_SIM` is defined) |

Everything except `main.cpp` and `hal_esp32/` is also compiled into the simulator
(`tools/build_core.py`), so the simulated robot runs this exact code.

## Build and upload
Requirements: Python 3.10+ and PlatformIO Core (`pip install platformio`). The first build downloads
the ESP32 toolchain (~0.5 GB). Versions are pinned in `platformio.ini`: platform `espressif32@8.0.0`
(Arduino-ESP32 3.3.12, ESP-IDF 5). Pick the environment for **your motor driver**:

| Environment | Driver | PWM |
|---|---|---|
| `esp32_l298n` (default, the purchased robot) | L298N: ENA/IN1/IN2, ENB/IN3/IN4 | 1 kHz |
| `esp32_tb6612` | TB6612FNG with STBY | 20 kHz |
| `esp32_drv8833` | DRV8833 / MX1508 (PWM on the IN pins) | 20 kHz |
| `*_debug` variants | same, plus serial event log and 20 Hz `TEL` telemetry | |

```bash
cd firmware
pio run -e esp32_l298n -t upload        # competition build; hold BOOT and tap EN if the upload does not start
pio run -e esp32_l298n_debug -t upload
pio device monitor                       # debug build only: event log + TEL telemetry at 115200 baud
pio run -t erase                         # wipes everything including the stored map/calibration (NVS)
```
Build flags: `-DMESH_VBAT_SENSE=1` once the 47k/10k battery divider is fitted on GPIO26.

Verified in this repository: the L298N, TB6612 and DRV8833 environments and the L298N debug build
**compile** (`esp32_l298n`: RAM 21.5 %, flash 28.1 %). **Not verified:** running on real hardware.

## Operating the robot (one button: START = GPIO0 / BOOT)
| Action (robot idle) | Effect |
|---|---|
| short press (< 1.5 s) | start the run implied by the stored stage: FRESH → dry run; DRY_PARTIAL → dry restart (relocalise); DRY_DONE → actual run |
| hold 1.5–4 s (status LED goes solid), release | sensor calibration sweep at the START line, plus a wiring polarity check |
| hold > 4 s (status LED flickers), release | clear the stored map and route (calibration kept) |
| any press while moving | operator stop: motors off, FAULT (the next press = restart) |

Status LED: slow blink = FRESH; double blink = dry run interrupted; mostly on = route stored;
fast blink = FAULT. Red LED = end box reached.

## Changing parameters
Edit `include/config.h`, rebuild, upload, then clear memory (hold > 4 s). Stored records from a
different firmware version are ignored automatically (`kFirmwareVersion` in `config.h`; bump it
whenever the record layout changes). **Rules:** code may not be changed after the robot is
deposited. Make sure the final parameters are flashed before the event.

## Timing budget (estimated; measure with the debug build)
| Work per 1 ms step | Estimate |
|---|---|
| 18 × analogRead (double conversion) | ~0.15–0.25 ms |
| core step (normal) | < 0.05 ms |
| Dijkstra at a node (≤ 384 states) | < 1 ms, once per junction |
| NVS save | tens of ms, only while stopped |
