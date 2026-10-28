# Power budget (purchased hardware; design estimate, measure on the robot)

Pack: 4 × 18650 in series (4S): 16.8 V full, 14.8 V nominal, stop at ~12.8 V.

| Consumer | Rail | Typical | Worst case | Source of the number |
|---|---|---|---|---|
| ESP32-WROOM-32, 240 MHz, radios off | 3V3 | 40 mA | 80 mA | Espressif typical active-mode figures; **MEASURE** |
| 5-ch TCRT5000 array (5 IR LEDs + LM339 + indicator LEDs) | 3V3 | 60 mA | 100 mA | ~12-15 mA per IR LED at 3.3 V (module resistor values vary); **MEASURE** |
| 2 × FC-51 | 3V3 | 30 mA | 40 mA | ~15-20 mA each; **MEASURE** |
| MPU6050 (GY-521) | 3V3 | 4 mA | 5 mA | datasheet ~3.6 mA + board LED |
| Red LED (GPIO15) + status LED | 3V3 | 0 mA | 10 mA | weakest pin drive or 330 Ω |
| **3V3 total** (ESP32 AMS1117) | | **~135 mA** | **~235 mA** | AMS1117 dissipation (5 − 3.3) V × 0.235 A = 0.4 W ✔ |
| L298N logic (5 V terminal) | 5 V | 30 mA | 40 mA | L298 logic supply current |
| LM2596 input at ~80 % efficiency | VBAT | 0.06 A at 14.8 V | 0.1 A | 5 V × 0.275 A / 0.8 / 14.8 V |
| Motors cruising (both) | VBAT | 0.2-0.4 A | — | same mechanical power as a 7.4 V pack at half the current; **MEASURE** |
| Motors accelerating / pivoting | VBAT | 0.6-1.2 A | ~2 A (both stalled, average at the 6 V duty cap) | BO stall ~1 A at 6 V (estimate); stall check stops the motors |
| **Battery total** | | **~0.4 A average** | **~2 A** | |

* **Runtime:** 18650 cells are typically 1500-2600 mAh; at ~0.4 A that is **3-5 h** of driving. Still
  charge fully before every match: without the battery-sense divider the firmware assumes 16.2 V.
* **Fuse:** 3 A (glass 5×20, twist holder) right after the pack. Normal peaks stay below ~2 A.
* **Wire:** 22 AWG (or thicker) for battery and motor leads; ordinary jumper wire for signals.
* **Voltage limit:** 16.8 V maximum anywhere on the robot, below the 24 V rule (p.5).
* **L298N drop:** ~2 V at 1 A, so the motors see at most ~14.8 V during a PWM on-pulse. The firmware
  caps the *average* motor voltage at `motor_max_avg_v` = 6 V (duty ≤ 6 / 16.2 = 37 %).
