# Architecture decision record

Each decision is compared on the project's ranking: (1) probability of completing both runs,
(2) reachable checkpoint score, (3) shortest-route correctness, (4) actual-run speed,
(5) hardware complexity, (6) power, (7) weight/size, (8) development time, (9) cost and
availability, (10) ease of testing and repair.

---
## ADR-1: One C++ "robot brain", run unchanged in the simulator
**Decision.** All behaviour (sensing, control, mapping, exploration, planning, persistence, the
competition state machine) lives in `firmware/src` as plain C++ with no heap and no Arduino calls.
`Robot::step(HalInputs, HalOutputs)` runs at 1 kHz. The hardware layer (`hal_esp32`) and the PC
simulator (`sim_api` + Python, loaded through ctypes) are two thin adapters around the same object.

**Why.** Score criteria (1)–(3) depend on navigation logic that cannot be tested on a real arena
in volume. Software-in-the-loop means every simulated match exercises the exact firmware code.
Several real defects were found this way and fixed:
* a derivative term that was reset on every control tick;
* false "both-sides" junction reads at 8 mm tracking error;
* wrong loop closures caused by a global match radius;
* checkpoint restarts that never re-joined the old map.

**Cost.** A host C++ compiler is needed (`pip install ziglang` provides one on Windows).

## ADR-2: Controller: ESP32-WROOM-32 DevKitC (team decision, replaces STM32F411)
| | ESP32-WROOM-32 DevKitC | STM32F411 Black Pill | Arduino Nano | RP2040 Pico |
|---|---|---|---|---|
| Price (India) | **₹381 (verified, Robocraze)** | ₹499 (verified, Probots) | ~₹300 | ~₹400 |
| ADC inputs usable here | 6 on ADC1 + ADC2 (radios off) = 9 needed ✔ | 10 ✔ | 8 ✘ (1 short) | 3 ✘ |
| Quadrature hardware | PCNT ✔ | timer encoder mode ✔ | ✘ (2 interrupt pins) | PIO ✔ |
| RAM for the map (~10 KB) | 320 KB ✔ | 128 KB ✔ | 2 KB ✘ | 264 KB ✔ |
| Non-volatile storage | NVS (wear-levelled) ✔ | flash sector, 1–2 s erase stall | EEPROM 1 KB ✘ | flash ✔ |
| ADC quality | non-linear near 0 V and above 2.5 V | linear | linear | linear |
| Radio | **must stay off by rule.** Firmware never starts it | none | none | none |
| Availability | everywhere | good | everywhere | good |

**Decision.** ESP32 DevKitC, chosen by the team. An STM32F411 HAL was written and compiled
(RAM 36.5 %, flash 16.4 %) before the switch, so the core is proven portable.
**Mitigations for the ESP32's weak points:**
* ADC non-linearity: the per-channel min/max calibration absorbs it. The line/floor decision only needs contrast, not linearity.
* ADC2 sharing with Wi-Fi: Wi-Fi is never started.
* Strapping pins: avoided or harmless loads (see the pin table).
* Boot-time pin toggling: STBY pull-down.
* Radio rule: no Wi-Fi/BT code is linked, `btStop()` is called at boot, and the referee can verify that nothing is advertising.

## ADR-3: Reflectance array: Pololu QTR-MD-08A at 3.3 V (original design; the built robot uses ADR-10)
* 8 channels at 8 mm pitch give a 56 mm sensor span. A 30 mm line covers at most 4 channels, so
  a run of 5 or more channels touching an array edge can only be a side branch.
* It runs at 3.3 V, so its output can never exceed the ESP32 pin rating. 5 V arrays would need dividers.
* 3.1 g; documented 125 mA.
* Alternatives: generic TCRT5000/ITR arrays (cheaper, often 5 V, height-sensitive), 12–16 channel
  arrays (need an ADC multiplexer). Rejected for complexity and risk.

## ADR-4: Motors: N20 6 V ~600 RPM with magnetic encoders; driver: TB6612FNG (original design; the built robot uses ADR-10)
* About 1 m/s with 34 mm wheels at the 6 V average limit. Faster runs are limited by 90° turns,
  not top speed (actual run on the sample arena: 17.6 s, 6.6 m).
* An encoder on every wheel is essential: distances define the map edges and the shortest route.
* TB6612FNG: 3 A peak covers the worst-case stall (2.24 A at 8.4 V). DRV8833's 2 A peak is marginal.
  Details in `electrical_design.md`.
* The battery (8.4 V) feeds the motors through PWM with an average-voltage clamp. No separate motor
  regulator is needed. Feed-forward uses the measured battery voltage, so behaviour is consistent
  as the battery discharges.

## ADR-5: Map as a metric graph with a discrete heading
* Lines meet only at 90°, so the heading is an N/E/S/W index that **never drifts**. Encoders measure
  distance only, giving a 1–3 % distance-proportional position error.
* Nodes are junctions, dead ends, START and the box entry. Corners are stored inside edges (they
  involve no decision), which keeps the graph small.
* Loop closure snaps the pose to the known node. The match radius grows with the distance driven
  since *that* node was last visited (70 mm + 3 %, capped at 150 mm). Two candidates at similar
  distances are treated as ambiguous: a duplicate node is harmless, a wrong merge is not.
* Rejected: grid/flood fill (there is no grid); pure LSRB string reduction (fails with loops, has no distances).

## ADR-6: Exploration policy
The candidates were measured in the batch study (see `exploration_strategy.md` and `results/`).
DFS with nearest-frontier backtracking was chosen, with the branch order and the START-distance
weight picked by expected total score. Wall following is kept only as an option. "Peek-and-prove"
(`defer_goal`) is implemented but **off** until the organisers confirm it is legal (Q4).

## ADR-7: End-box detection by distance-persistence
* All channels except at most one must be white continuously for **70 mm of travel**. A 30 mm
  junction cannot do that.
* Then a 140 mm confirmation drive with the LED on puts the whole body inside the box.
* If white is lost during the confirmation drive, the LED goes off and the robot reverses back to
  the junction it overshot.
* Persistence is measured in distance, not time, so detection does not depend on speed.

## ADR-8: Stage persistence instead of operator mode switches
* The robot stores stage, calibration, map and route in NVS (CRC-protected, firmware-version-tagged).
* A short press starts whatever the stored stage implies: FRESH → dry run; DRY_PARTIAL → dry restart
  with relocalisation; DRY_DONE → actual run.
* No information about the arena is entered by the operator (rule: no feeding arena information).
* A corrupt or missing record makes the robot calibrate itself and search; it never runs a
  meaningless route.

## ADR-9: Restart handling by anchored relocalisation
* The referee places the robot on the last checkpoint crossed, and checkpoints are junctions in the
  sample arena. The restart point is therefore an **old junction** with an unknown identity and heading.
* Candidates: (old junction, exit). They are filtered by consistency of the newly mapped nodes.
  If several remain when the box is reached, the most recently visited junction is chosen.
* If the map stops being consistent without a restart (for example after a bump), the robot rolls
  back to its last verified map and relocalises the same way.
* Update (v1.1): START is also an anchor candidate (a restart before any checkpoint), an anchor
  needs at least two re-observed nodes and must be unique or beat the runner-up by two, and a free
  (un-anchored) alignment needs three. A checkpoint can lie on a line the old map never reached, and
  one coincidental match then looked unique.

## ADR-10: Purchased hardware (team decision, v1.1)
The team bought: ESP32 DevKit V1, a 5-channel digital TCRT5000 array, 2 FC-51 IR modules, 2 BO
500 RPM motors **without encoders**, an L298N, an LM2596 buck, an MPU6050, a 2WD chassis kit, a
4S 18650 pack (with a TP4056), a red LED, a fuse and a rocker switch. The design adapts to them:

| Topic | Original design | Purchased hardware | How the firmware copes |
|---|---|---|---|
| Distance | quadrature encoders (~1400 counts/rev) | none | first-order speed model of the commanded wheel speeds (`encoder_mode = 2`); calibrated in Phase B; optional slot encoders (`encoder_mode = 1`) |
| Heading / turn angle | encoder difference | MPU6050 gyro Z | bias averaged at every standstill ≥ 300 ms; wiring check in the calibration sweep |
| Motor imbalance | wheel-speed PI loop | none | left/right trim learned from the steady steering on straight lines |
| Line sensing | 8 analog channels, 8 mm pitch | 5 digital channels, 15 mm pitch | per-channel polarity learned in calibration; capture tolerance ≥ 0.6 × pitch; centre window ≥ 1.2 × pitch |
| Side branches | array edge channels | 2 wing sensors at ±45 mm | a side counts as a branch only if white appears well beyond the tracked line (`branch_side_margin_mm`), so an off-centre robot does not report phantom branches |
| Stall | no encoder motion for 450 ms | not observable | 3200 mm of modelled travel without any track event ⇒ STALL |
| Driver | TB6612 (MOSFET) | L298N (~2 V drop) | 1 kHz PWM build; feed-forward absorbs the drop |
| Battery | 2S LiPo, measured | 4S 18650, not measured (no resistors) | assumes a full pack (16.2 V); 47k/10k divider recommended |

**Consequences.** Simulation on 100 held-out mazes + the sample (results/test_summary.md) shows
the no-encoder robot is workable, with lower margins than the original design; the gap to slot
encoders is measured there. Physical risks that simulation cannot settle: real BO motor
repeatability, FC-51 behaviour as a down-looking line sensor, gyro vibration noise, and whether the
organisers accept the purchased chassis plate (rule p.5) and an ESP32 with its radios switched off.
