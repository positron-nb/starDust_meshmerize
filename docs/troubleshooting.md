# Troubleshooting (purchased hardware: BO motors without encoders, L298N, MPU6050)

| Symptom | Likely cause | What to do |
|---|---|---|
| Nothing happens on START, status LED blinks fast | FAULT state (see the debug log): uncalibrated, or a previous fault | Hold START 1.5-4 s to calibrate. Short press = restart. |
| ESP32 resets or does not boot when the battery is switched on | LM2596 not at 5 V, or the L298N 5V-EN jumper still fitted (its regulator overheats above 12 V) | Remove the 5V-EN jumper; set the LM2596 to 5.0 V with a multimeter |
| `TEL … gyro=…(none)` | MPU6050 not answering | SDA→D21, SCL→D22, VCC→3V3, GND. The firmware then falls back to the speed model for heading (much worse) |
| "CAL robot not rotating" | a motor not running, or both motors push the same way | ENA/ENB jumper caps must be removed; check IN1-IN4 wires; swap one motor's wires if both turn the same way |
| "CAL robot turned RIGHT … when told to turn left" | both motors reversed, or the gyro sign is wrong | Phase 0.3 of the calibration guide (gyro sign); else swap both motors' wires |
| "CAL bad channels mask=0x.." | a sensor channel never changed | Bits 0-4 = D1-D5, 5/6 = left/right wing. Check its wire, adjust its pot, check its height (5-10 mm) |
| One motor never turns, L298N gets hot | ENA/ENB jumper still fitted, or PWM frequency too high | Remove the caps; use the `esp32_l298n` build (1 kHz PWM) |
| Robot drifts to one side / `trim=` beyond ±0.25 | one motor or wheel much weaker, binding gearbox | Check the wheel turns freely; swap the motors to test; replace a bad motor |
| Robot oscillates on straights | `line_kp` too high or `line_kd` too low | Lower kp by 20 %; raise kd |
| Junction distances (`EVENT JUNCTION … at s=`) off by more than 3 % | speed model not calibrated, or the battery voltage differs from `vbat_nominal_v` | Calibration Phase B; charge fully; fit the battery-sense divider (`MESH_VBAT_SENSE=1`) |
| Turns overshoot / "TURN failed to capture the line" | pivot too fast for the BO motors, or the pots set badly | Lower `turn_omega_deg_s`; re-adjust the IR pots; check the array height |
| Junctions read with a wrong L/R | wing sensor misses or false triggers | Adjust the FC-51 pots over the real floor; shield them from strong light |
| "TURN no line … exit removed" or "MAP node … has no … line" now and then | a branch was misread and the firmware corrected it | Fine if rare. If frequent, adjust the wing sensors |
| "MAP inconsistent … rolling back" or "MAP arrival error" | distances wrong (speed model) or the robot was bumped | Phase B; lower acceleration; check for wheel slip |
| "STALL: … modelled travel without a junction or corner" | a wheel jammed, or a straight line longer than `stall_no_event_mm` (3200 mm) | Free the wheel. Only raise the limit if your practice arena is bigger than the official 2.3 m |
| LED lights at a junction (false goal) | `goal_confirm_mm` too small, or a shiny floor reading white | Raise to 80-100 mm; re-calibrate on the actual floor |
| Robot stops at the box edge, LED never on | the box reads darker than the vinyl | Re-calibrate including the box |
| Red LED very dim | no series resistor: the firmware limits the pin current | Normal without a resistor; add 330 Ω if you want, or use a brighter LED |
| Actual run takes the wrong turn at the start | stage/route lost, or a different build | Log "STORE …". Re-run the dry run. Never re-flash between runs (that is also a rule) |
| "STORE … FAILED" | NVS full or corrupted | `pio run -t erase` (wipes calibration too), then recalibrate |
| A motor twitches at power-on | GPIO14/GPIO5 toggle during the ESP32 boot | Harmless |

## Reading the log
Every important decision is logged with a timestamp:
* `EVENT JUNCTION L1 S0 R1 at s=… (bar … mm, entry offset … mm)`
* `MAP new node`, `MAP loop closure (odometry error …)`, `MAP node … gains exits`
* `NAV node n: take L`, `EXP heading for frontier …`
* `ROUTE … PROVABLY SHORTEST`, `RELOC aligned`, `FAULT …`, `STALL: …`
* `EVENT GOAL … motor trim +0.042`

The simulator prints the same lines, so a real run can be compared with a simulated one.
