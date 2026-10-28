# Testing and validation

## Levels of evidence (kept strictly apart)
| Level | What it proves | Status |
|---|---|---|
| **Unit tests**: C++ planner, graph, persistence, Python scoring | algorithms correct on hand-made cases | ✔ automated (`simulator/tests/test_planner_graph.py`, `test_scoring.py`, `test_consistency.py`) |
| **Closed-loop simulation tests**: firmware core + simulated physics | behaviour in scenarios (turns, junctions, faults, restarts, persistence) | ✔ automated (`test_sim_scenarios.py`) |
| **Randomised simulation batches** | completion, shortest-route and score statistics | ✔ `tools/run_batch_tests.py`, results in `results/` |
| **Firmware compile** (ESP32: L298N, TB6612, DRV8833, L298N debug) | code builds for the target with the pinned toolchain | ✔ `pio run -e esp32_l298n`, `pio run -e esp32_l298n_debug` |
| **CAD checks** (original N20 design only) | geometry fits; no unexpected collisions | ✔ `hardware/cad/robot_cad.py` |
| Hardware-in-the-loop | — | ✘ not done (no hardware) |
| Bench tests (sensors, polarity, odometry, current) | — | ✘ to do: `calibration_guide.md` Phases A–B |
| Practice-arena tests | — | ✘ to do: Phases C–G |

## Commands
```bash
python -m pytest simulator/tests                            # all automated tests (~4-5 min)
python tools/run_batch_tests.py --mazes 100 --seed0 500 --include-sample --profile real  --label final_real
python tools/run_batch_tests.py --mazes 100 --seed0 500 --include-sample --profile harsh --label final_harsh
python tools/run_batch_tests.py --mazes 100 --seed0 500 --include-sample --profile real --label slot_encoders_real --sim encoder_mode=1
python tools/analyze_results.py results/benchmark_results.csv
cd firmware && pio run -e esp32_l298n && pio run -e esp32_l298n_debug
python hardware/cad/robot_cad.py
```
Machine-readable outputs: `results/pytest_report.xml` (JUnit), `results/benchmark_results.csv`,
`results/policy_study.csv`, `results/batch_summaries.jsonl`.

## Required test list → where it is covered
| # | Requirement | Test(s) |
|---|---|---|
| 1 | straight line following | `test_straight_line_following_and_goal` (max deviation < 12 mm) |
| 2–3 | 90° left/right turns | `test_corners[True/False]` (corner stored inside the edge with the right turn sign) |
| 4 | T-junction | `test_t_junction_and_dead_end_recovery` |
| 5 | 4-way crossing | `test_four_way_crossing` |
| 6 | dead end + recovery | `test_t_junction_and_dead_end_recovery` |
| 7–8 | loops, repeated junctions | `test_loops_and_repeated_junctions` (loop closures, no node explosion) |
| 9–10 | multiple routes, shortest chosen | `test_multiple_routes_shortest_selected`; unit `test_shortest_*`; batch shortest % |
| 11 | equal-distance tie-break | unit `test_equal_length_tie_break_by_turns`, `test_length_dominates_turn_tiebreak` |
| 12 | sensor noise | `test_sample_arena_harsh_noise`; batch `final_harsh` |
| 13 | surface reflectance | `test_dull_line_and_shiny_floor` |
| 14 | encoder / motor polarity mistake | `test_polarity_mistake_detected[bo-motor, n20-encoder, n20-motor]` (caught at calibration: gyro check without encoders) |
| 15 | wheel-speed mismatch | `test_wheel_mismatch_and_odometry_error` (±15 % motor gain, no encoders: learned motor trim) |
| 16 | accumulated odometry error | same (3 % / −2 % wheel diameter error); batch `final_harsh` |
| 17 | no false goal at intersections | `test_no_false_goal_at_crossings`; batch false_goal_led = 0 |
| 18 | missed goal | batch `missed_goal_runs`; `test_dull_line_and_shiny_floor` (box reflectance 0.75) |
| 19 | genuine end zone + LED, stop inside | `test_end_zone_led_and_stop_inside_box` (whole body inside) |
| 20 | line loss and recovery | `test_line_loss_recovery_after_bump` (70° kick) |
| 21 | dry-run time limit | `test_dry_run_time_limit` |
| 22 | actual-run time limit | `test_actual_run_time_limit` |
| 23 | combined-time accounting | `test_dry_overrun_reduces_actual_time`, `test_combined_time_limit_constants` |
| 24 | restart limits and timing | `test_restart_limit_three`, `test_dry_restart_from_checkpoint_relocalises[15 s: from START, 35 s: from a checkpoint]` (time includes handling) |
| 25 | route persistence across stages | `test_route_survives_power_cycle`; every batch match power-cycles between runs |
| 26 | missing / corrupted route | `test_corrupted_storage_falls_back_safely`; unit `test_persistence_roundtrip_and_corruption` |
| 27 | checkpoint score accounting | `test_sample_arena_checkpoints_and_score`, `test_checkpoints_counted_once_each` |
| 28 | penalties | `test_penalties_subtract` (values unknown: Q7) |
| 29 | different topologies | `test_generated_topologies[4 families]`; batches `tree_only`, `loop_heavy` |
| 30 | emergency stop / controller fault | `test_operator_emergency_stop`, `test_stall_detected_and_motors_stopped[bo, n20]` |
| + | displaced robot / map inconsistency | `test_displaced_robot_detects_inconsistent_map` |
| + | optional slot encoders | `test_slot_encoder_mode_completes_sample[realistic, harsh]` |
| + | pin map = firmware, geometry consistency | `test_consistency.py` |

## Results
See `results/test_summary.md` for the numbers from the latest run and their honest interpretation.
