# Changelog

## 1.1.0 (2026-10-28): purchased hardware
The team bought its parts (ADR-10): ESP32 DevKit V1, 5-channel digital TCRT5000 array, 2 FC-51 wing
sensors, BO 500 RPM motors **without encoders**, L298N, LM2596, MPU6050, 2WD chassis kit, 4S 18650
pack, TP4056, fuse, rocker switch, red LED. Firmware, simulator, tests and documents were adapted.

### Hardware support
* Odometry modes: quadrature (`encoder_mode` 0), slot-disc (1) and **no encoders** (2: first-order
  model of the commanded wheel speeds). Heading and turn angle from the MPU6050 gyro, with the bias
  re-estimated at every standstill. Gyro wiring check in the calibration sweep.
* Wing sensors, per-channel polarity learned during calibration (digital comparator outputs read
  through the ADC), driver builds for L298N (1 kHz PWM), TB6612 and DRV8833.
* Battery sensing is optional (`MESH_VBAT_SENSE`, default off: no resistors). Without it the
  firmware assumes a full 4S pack (16.2 V). The red LED pin uses the weakest drive strength in case
  it is wired without a resistor.
* New pin map for the 30-pin DevKit V1; circuit diagram `docs/Circuit_Diagram.pdf` generated from
  the pin table.

### Fixes found in simulation of the new hardware
* Turns never captured the line: a line wider than the 15 mm pitch makes the digital array jump
  between ±7.5 mm, so the 7 mm capture tolerance was never met. Tolerance is now ≥ 0.6 × pitch, and
  the "centre" window ≥ 1.2 × pitch (a slightly tilted robot sees its line only on the channels
  next to the middle one, which made straight exits disappear).
* Phantom side branches: an off-centre robot whose line already lit an edge channel reported a branch
  on both sides when a one-sided branch filled the rest of the array. A side now counts only if
  white appears beyond the tracked line (`branch_side_margin_mm`, slow-averaged line position).
* A dead end right after leaving a junction straight now removes that phantom exit and chooses
  again; an arrival far (> 150 mm) from the expected node now counts as a map inconsistency; the
  relocalisation seed uses the exits actually observed.
* No-encoder motor imbalance: left/right trim learned from the steady line-follow steering.
  A 15 % motor mismatch went from 3 restarts and a failed actual run to a clean shortest-path run.
* No-encoder stall: 3200 mm of modelled travel with no junction, corner or dead end ⇒ STALL
  (previously a jammed robot pushed for 25 s until the referee stopped it).
* Known-node arrivals now keep newly observed exits (a restart point's side lines were never added,
  which caused NO_FRONTIER after a restart).
* Slot encoders (`encoder_mode` 1) are used for distance only. Their ~5 mm counts made the wheel-speed
  loop so laggy that turns failed in almost every simulated match; motor balance now comes from the
  learned trim, as without encoders. A commanded stop no longer brakes against the slot speed: its
  sign is only assumed, and once a wheel reversed the brake ran away (robot spun off the line).
* Relocalisation: START is an anchor candidate; anchors need ≥ 2 matches and must be unique or
  beat the runner-up by 2; free alignments need 3. One coincidental match had produced wrong merges.

### Referee model
* The simulated referee now judges "shortest path" by the **route** (sequence of maze points the
  robot passed) instead of the axle odometer length, which depended on robot size.

### Results
* N20-era benchmark results are archived as `results/benchmark_results_n20_v1.0.csv`; the BO
  hardware results are in `results/benchmark_results.csv` and `results/test_summary.md`.

## 1.0.0 (2026-10-26)
First complete version: firmware core, ESP32 hardware layer, simulator, tests, CAD, documentation.

### Changes during development (all found by tests or simulation)
* **Controller switched from STM32F411 Black Pill to ESP32-WROOM-32 DevKitC** (team decision). The
  STM32 HAL compiled (RAM 36.5 %, flash 16.4 %) before it was replaced. The robot core needed no change.
* Fixed: the line-follow derivative state was reset every control tick, so `line_kd` had no effect.
  Retuned gains: junction-entry error went from 5.6 mm RMS / 18.6 mm max to 1.8 mm / ~4 mm (sample arena, realistic noise).
* Fixed: "all white" with 7 of 8 channels was read as a cross line on *both* sides. Now all 8 are
  required; the end-box test keeps its own 7-of-8 tolerance.
* Fixed: the end box was only detected while a crossing was in progress; it is now tested on every sample.
* Fixed: wrong loop closures that came from a match radius growing with the *total* distance driven.
  The radius is now per node (distance since that node was last visited), with a tighter cap
  (150 mm), a self-loop geometry check and an ambiguity guard. Eliminated the MAP_FULL failures seen
  on 2 of 61 random mazes.
* Fixed: after a dry-run restart the robot never re-joined the old map when it only explored new
  ground. Added anchored relocalisation (the restart junction must be an old junction), forced at
  the box by recency if still ambiguous. The START-based map is preserved if re-joining fails.
* Added: rollback and self-relocalisation when the map becomes inconsistent without a restart (bump/slip).
* Added: wiring polarity check during calibration; automatic calibration when none is stored.
* Added: frontier selection weighted by distance from START (`explore_start_weight`), chosen from the policy study.
