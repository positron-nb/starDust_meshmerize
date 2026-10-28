# Carrier board (purchased hardware)

No PCB layout files are included. The purchased parts are all modules (ESP32 DevKit V1, L298N,
LM2596, IR array, FC-51, MPU6050), so the robot is wired with jumper wires plus one small
perfboard (about 50 × 70 mm) that carries the ESP32 on female headers, the common GND and 3V3
distribution, and the optional resistors and capacitors. The drawing and the wire-by-wire list are
in **`docs/Circuit_Diagram.pdf`**; the GPIO numbers come from `hardware/pin_mapping.csv` (checked
against `firmware/src/hal_esp32/pins.h` by `simulator/tests/test_consistency.py`).

## Netlist
| Net | Connections |
|---|---|
| BAT+ | 4S pack + → 3 A fuse → rocker switch → **VBAT** |
| VBAT | L298N 12V terminal; LM2596 IN+; (optional) 470 µF 35 V to GND at the L298N; (optional) 47 kΩ to VSENSE |
| GND | pack −; L298N GND; LM2596 IN− and OUT−; ESP32 GND; IR array, FC-51 ×2 and MPU6050 GND; red LED cathode |
| 5V | LM2596 OUT+ (set to 5.0 V first); ESP32 VIN; L298N 5V terminal (5V-EN jumper removed) |
| 3V3 | ESP32 3V3 pin; IR array VCC; FC-51 ×2 VCC; MPU6050 VCC |
| VSENSE (optional) | 47 kΩ from VBAT, 10 kΩ to GND, ESP32 GPIO26 (`MESH_VBAT_SENSE=1`) |
| IR D1-D5 | ESP32 GPIO36, 39, 34, 35, 32 |
| WING L / R | FC-51 OUT → ESP32 GPIO33 / GPIO25 |
| SDA / SCL | MPU6050 → ESP32 GPIO21 / GPIO22 |
| ENA, IN1, IN2 | ESP32 GPIO23, 19, 18 → L298N (ENA jumper removed) |
| ENB, IN3, IN4 | ESP32 GPIO14, 5, 4 → L298N (ENB jumper removed) |
| MOTOR L / R | L298N OUT1/OUT2 → left BO motor; OUT3/OUT4 → right BO motor (100 nF across each, optional) |
| LED | ESP32 GPIO15 → (330 Ω recommended) → red LED anode; cathode → GND |
| START | on-board BOOT button (GPIO0): nothing to wire |
| ENC L / R (optional) | LM393 slot sensor D0 → ESP32 GPIO16 / GPIO17 (VCC 3V3), then `encoder_mode = 1` |

## Build tips
* Use female headers for the ESP32 so it can be swapped.
* Keep the motor wires (L298N outputs) twisted and away from the sensor wires.
* Do the multimeter checks in `docs/wiring_and_pinout.md` before connecting the ESP32.

The original design's carrier board (TB6612, MP1584, reverse-polarity MOSFET, 2S LiPo) is
described in the v1.0.0 package (`dist/meshmerize-robot-v1.0.0.zip`) and in
`docs/architecture_decisions.md` ADR-4.
