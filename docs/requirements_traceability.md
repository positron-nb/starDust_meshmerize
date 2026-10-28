# Requirements traceability

How each requirement (the official PDF, plus the project brief) maps to design, code and evidence.
Evidence key: **U** unit test, **S** closed-loop simulation test, **B** batch benchmark,
**C** compiled, **CAD** automated CAD check, **D** design document only, **HW** needs hardware validation.

| Requirement | Source | Design / code | Evidence |
|---|---|---|---|
| Follow a 30 mm white line on black | PDF p.1 | `line_sensor.*`, `motion_controller.cpp` FOLLOW | S `test_straight_line_following_and_goal` (deviation < 12 mm); HW Phase C |
| 90° turns, left/right | PDF p.1 | pivot + line capture, `Robot::startTurn` | S `test_corners[left/right]` |
| T-junctions, crossings, dead ends, loops | PDF Fig.1 | `junction_detector.*`, graph, explorer | S `test_t_junction…`, `test_four_way_crossing`, `test_loops…`, `test_generated_topologies` |
| Unknown altered arena (no hard-coding) | PDF p.1 | generic graph exploration; sample only used for tests | B: 80 random mazes per policy |
| End box 400 × 400; LED as soon as it is reached | PDF p.1, 4, 5 | `goal_detector.*` (70 mm persistence + 140 mm confirmation); GPIO15 | S `test_end_zone_led_and_stop_inside_box`, `test_no_false_goal_at_crossings`; B false_goal_led = 0 |
| Dry run: store the route; actual run: use it | PDF p.4 | `persistence/storage.*`, NVS, `ROUTE_PLANNING`, route execution | S `test_route_survives_power_cycle`; U persistence round trip |
| Shortest path in the actual run | PDF p.4, 6 | Dijkstra with turn tie-break, `planner.*` | U planner tests; B shortest-route rate |
| Time limits 3:00 / 2:30 / 5:30, overrun deduction | PDF p.4 | `scoring/score.py`, `match.py` | U `test_dry_overrun_reduces_actual_time`; S time-limit tests |
| ≤ 3 restarts; dry restart at checkpoint; actual restart at START; clock never paused; no arena info | PDF p.4 | stage machine, relocaliser, referee model | S `test_dry_restart_from_checkpoint_relocalises`, `test_restart_limit_three` |
| Scoring formula A+B+C+S+L+T−P | PDF p.6–7 | `score.py` | U `test_scoring.py` |
| Size ≤ 220 mm cube | PDF p.5 | CAD 151.5 × 106 × 63 mm | CAD `fit_report.md`; HW measure |
| One start switch (+ on-board switch) | PDF p.5 | GPIO0 button; power switch | D; HW |
| Red LED | PDF p.5 | GPIO15 | S (LED state); HW |
| No arena damage, nothing left behind | PDF p.5 | ball caster, strap, clearance ≥ 3 mm | CAD clearance check; HW |
| On-board power, ≤ 24 V | PDF p.5 | 4S 18650, 16.8 V max | D `electrical_design.md` |
| No Lego / ready-made mechanism | PDF p.5 | custom printed chassis | D |
| 1-minute calibration | PDF p.5 | ~1.5 s sweep calibration + wiring check | S `calibrate()` in every match; HW |
| No code change after deposit | PDF p.5 | all modes by button + stored stage; parameters fixed at build time | D |
| No Wi-Fi/Bluetooth use, no laptop | PDF p.6 | radios never started, `btStop()` | C (no Wi-Fi/BT library linked); HW verify with a phone scan |
| Encoder-based motion, not fixed delays | brief §7 | odometry, profiles, pivot capture; the loop never blocks | C; S |
| Config in one file | brief §7 | `include/config.h` X-macro | C |
| State machine with error/recovery states | brief §7 | `app/robot.*`, `system_architecture.md` | S fault tests |
| Pin map = firmware = wiring diagram | brief §5 | `pin_mapping.csv` → `pins.h`, generated SVG | U `test_consistency.py` |
| Power budget, driver sizing, 3.3 V compatibility | brief §5 | `electrical_design.md`, `power_budget.md` | D; HW measure stall current |
| Simulator must not use hidden maze info | brief §11 | robot only gets HalInputs via `sim_api` | code structure (no other API reaches the robot) |
| Randomised batch, honest metrics | brief §12 | `tools/run_batch_tests.py`, `results/` | B |
