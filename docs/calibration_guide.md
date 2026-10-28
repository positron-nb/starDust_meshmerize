# Calibration and tuning guide (purchased hardware)

Hardware: ESP32 DevKit V1, 5-channel digital IR array + 2 FC-51 wing sensors, BO 500 RPM motors
**without encoders** on an L298N, MPU6050 gyro, 4S 18650 pack. Wiring: `docs/Circuit_Diagram.pdf`.

Work through the phases **in order**. Do not move on until the acceptance criteria are met.
Use the debug build on the practice arena only:
```bash
cd firmware
pio run -e esp32_l298n_debug -t upload
pio device monitor
```
The event log and a 20 Hz `TEL` line print over USB:
`TEL <ms> s=<mm> th=<deg> gyro=<dps> enc=L,R pos=<mm> on=<n> v=<5 array>|<2 wings> raw=... vbat=... trim=... st=<state>`.

Every parameter mentioned below lives in `firmware/include/config.h`. After changing one,
re-flash, then **hold START for more than 4 s** (clear memory). The competition build is
`esp32_l298n` (no serial logging).

> **How this robot knows where it is.** Without wheel encoders the firmware *models* the wheel
> speeds from the motor voltages it applies (`ff_mm_s_per_v`, `ff_static_v`, `motor_tau_ms`) and
> takes turn angles and heading from the MPU6050 gyro. Distances between junctions are therefore
> only as good as Phase B. It also learns the left/right motor imbalance while it follows straight
> lines (`trim=` in the log). Slot encoders (BOM item 22, `encoder_mode = 1`) replace the speed model
> with measured distance.

## Phase 0: Bench (robot on a stand, wheels in the air)
1. Multimeter checks from `docs/wiring_and_pinout.md`; LM2596 set to 5.0 V.
2. Power up. Status LED blinks slowly (stage FRESH). `TEL` shows `gyro=` near 0 (not `(none)`).
   `(none)` means the MPU6050 did not answer: check SDA→D21, SCL→D22, VCC→3V3.
3. Turn the robot by hand on the table: `gyro` goes **positive for a left (anticlockwise) turn**.
   If it goes negative, set `gyro_sign = -1`.
4. Battery: there is no battery sensing unless the 47k/10k divider is fitted (`vbat=0.00`).
   Charge fully before every session; the firmware assumes 16.2 V.

## Phase A: Sensors
1. **Pots first.** Put each sensor over the black floor, then over the white line, and turn its
   blue pot until the module's indicator LED switches cleanly between the two (it does not matter
   which way round). Do the same for both FC-51s, mounted pointing **down** 5-10 mm above the floor.
2. **Calibration sweep:** put the robot on the line at START and **hold START for 1.5-4 s**, then
   release. The robot pivots ±50° over the line, learns each channel's min/max **and polarity**,
   and checks the wiring with the gyro:
   * `CAL robot not rotating`: a motor is not running or both motors push the same way. Check the
     L298N wiring and the ENA/ENB jumpers (must be removed).
   * `CAL robot turned RIGHT … when told to turn left`: swap the two wires of **both** motors, or
     flip `left_motor_invert` and `right_motor_invert`. If the gyro sign is wrong instead, fix Phase 0.3.
   * `CAL bad channels mask=0x..`: that channel never changed. Bit 0-4 = D1-D5, bit 5/6 = left/right
     wing. Check its wire, its pot, and its height.
3. Slide the robot sideways across the line by hand and watch `pos=` and `v=`. `pos` must go from
   about −30 to +30 mm as the line moves from the robot's left to its right. If it runs the other
   way, set `sensor_flip = 1`. The left wing (first value after `|`) must react on the left side.
4. **End box:** put the robot on a white sheet bigger than the robot. All 5 array channels and both
   wings must read white (1000). A glossy box can read dark: re-calibrate on the actual arena during
   the 1-minute calibration.
5. **Measure and enter** (calipers/ruler): `sensor_pitch_mm` (centre to centre of the array
   sensors, ~15), `wing_lateral_mm` (wing sensor to robot centreline, design 45),
   `sensor_forward_mm` (wheel axle to the sensor line, design 100), `track_width_mm`
   (wheel centre to wheel centre), `wheel_diameter_mm`.

## Phase B: Speed model (the no-encoder odometry)
1. **Straight test.** Tape a straight 30 mm line with a T-junction at each end, exactly 1500 mm
   apart. Let the robot run the dry run from one T: the log prints
   `EVENT JUNCTION … at s=<measured>` at the second T.
2. If `s` reads more than the true 1500 mm, the robot is slower than the model thinks:
   set `ff_mm_s_per_v` = old × (1500 / s). If it reads less, the same formula raises it.
   Repeat until 5 runs are within ±2 %.
3. **Static voltage:** command the slowest exploration speed (`explore_speed_mm_s` = 200) on the
   same line. If the robot hesitates or stops at low speed, raise `ff_static_v` by 0.1 V.
4. **Motor trim:** after a straight run, the `EVENT GOAL … motor trim` log line (or `trim=` in TEL)
   shows the learned imbalance. Values within ±0.15 are normal for BO motors; beyond ±0.25 one motor
   or wheel is binding: check it.
5. **Acceptance:** junction-to-junction distances within ±3 % over 10 runs; pivots end with the line
   captured (`TURN failed` never appears).

## Phase C: Line following and turns
1. Start at `explore_speed_mm_s` = 300 on a 2 m straight with an oval end. Increase `line_kp` until
   it oscillates slightly, back off 30 %, then add `line_kd` until the oscillation dies within one
   wavelength. Simulated defaults: kp 0.09, kp/(m/s) 0.15, kd 0.004.
2. **Acceptance (log "entry offset"):** junction entry offset below 8 mm (one digital step is 7.5 mm).
3. Left and right 90° turns separately, 20 each: no "TURN failed". If turns overshoot, reduce
   `turn_omega_deg_s` (BO motors react slower than N20s) or `turn_alpha_deg_s2`.
4. U-turn at a dead end, 20 times.

## Phase D: Junctions and exploration
1. Build a small practice maze with tape: T, crossing, corner, dead end, a loop, and a 400 × 400 box.
2. Run the dry run 10 times. Each log must show all junctions with the correct `L S R`, loop
   closures with an error below 70 mm, and "MAP validation … 0 problems". Occasional
   `TURN no line … exit removed` or `MAP node … has no … line` lines are the firmware correcting a
   misread branch; frequent ones mean the wing sensors or pots need adjusting.
3. **Acceptance:** 10/10 dry runs complete; the route matches the true shortest route.

## Phase E: End box
1. Drive slowly across every junction type 20 times. There must be **no** "EVENT GOAL" and the LED
   stays off.
2. Enter the box 20 times from different headings. The LED comes on within about 70 mm of entry and
   the robot stops fully inside.
3. False goals: increase `goal_confirm_mm` (80-100 mm). Missed goals: re-calibrate on the box.

## Phase F: Actual-run speed
Raise `actual_speed_mm_s` in steps: 600 → 800 → 1000. Move up only after **10 clean runs** at the
current level (no RECOVERY, no "EVENT ignored"). Without encoders, braking distances come from the
speed model, so redo Phase B if turns at speed start late.

## Phase G: Robustness
* Repeat C-F with a full pack and with a pack at ~14 V. Large differences mean the battery-sense
  divider is worth fitting (`MESH_VBAT_SENSE=1`).
* Different lighting (phone torch at 30 cm, room lights off): FC-51s are the most sensitive.
* 3 consecutive matches with a power cycle between dry and actual runs.
* Judge-style restart: stop the robot (press START while moving), place it on the last checkpoint
  facing onwards (or on START if none was crossed), press START. The log must show `RELOC aligned`
  before the box.
* Jam test: hold one wheel while it follows a line. Within a few seconds the log shows
  `STALL: … modelled travel without a junction or corner` and the motors stop.

## Competition day (1-minute calibration window)
1. Place the robot on START, hold START for 2 s, release. The robot sweeps (≈1.5 s).
2. If the record is from practice: hold START for more than 4 s (status LED flickers fast), release
   to clear the stored map (calibration is kept). Slow blink = stage FRESH.
3. Leave the robot on START. Wait for the organiser's signal, then press START once.
