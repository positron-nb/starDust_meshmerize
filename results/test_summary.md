# Test summary (generated 2026-10-29 by tools/make_test_summary.py)

All numbers below come from **software**: unit tests, closed-loop simulation and firmware compilation. No physical robot has been built or tested yet.

## Automated tests (`python -m pytest simulator/tests`)

* **61 tests, 0 failures, 0 errors, 0 skipped** (255 s). JUnit report: `results/pytest_report.xml`.

## Firmware compilation

* `pio run -e esp32_l298n` (competition build for the purchased robot): SUCCESS. RAM 21.5 % (70 552 B), flash 28.1 %. Arduino-ESP32 3.3.12.
* `pio run -e esp32_l298n_debug`, `esp32_tb6612`, `esp32_drv8833`: SUCCESS.
* Host build of the same core (zig c++ 0.16, `-Wall -Wextra`): no warnings.
* Not done: flashing and running on an ESP32 (no hardware available).

## CAD checks (`python hardware/cad/robot_cad.py`, original N20 design)

* 10/10 automated geometric checks pass (fits 220 x 220 x 220 mm box: envelope 151.5 x 106.0 x 63.1 mm; footprint near 150 x 140 mm target: 151.5 x 106.0 mm).
* Estimated mass 179 g. Rendered geometry only.

## Randomised simulation benchmarks (`results/benchmark_results.csv`)

Each run is a complete match with the firmware core in the loop: calibration → dry run → power cycle → actual run, judged by the simulated referee with the official formula. Simulated hardware = the purchased robot: BO motors without encoders (speed model), 5-ch digital array (15 mm pitch) + 2 wing sensors, MPU6050 gyro, 4S pack without voltage sensing (firmware assumes 16.2 V). Shortest path is judged by the route driven (sequence of maze points), not the odometer. Maze generator: 7 × 7 irregular grid, ≥ 190 mm spacing, 25 % extra loop edges, box at a leaf, START at the farthest leaf, 4 checkpoints on junctions (seeds 500+ are **not** the seeds used for tuning, 100–179).

| run set | description | runs | dry % | actual % | shortest % | proven % | checkpoints | dry s mean / worst | actual s mean | score mean | score p10 | score min | restarts | false goals | line lost | faults |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| final_real | purchased robot (no encoders), realistic imperfections, 100 held-out mazes + sample | 101 | 99 | 99 | 95.0 | 85.1 | 3.58 | 68.1 / 109.9 | 8.5 | 398.9 | 386.7 | 100.0 | 4 | 0 | 0 | 0 |
| final_harsh | purchased robot, harsh imperfections (unequal motors, noisy/dull sensors, gyro drift, 10 % speed-model error) | 101 | 93 | 89 | 83.2 | 76.2 | 3.42 | 70.6 / 320.1 | 8.2 | 363.2 | 233.2 | -5.1 | 20 | 1 | 4 | 3 |
| final_ideal | purchased robot, no imperfections | 41 | 100 | 100 | 100.0 | 92.7 | 3.68 | 72.3 / 103.5 | 7.9 | 406.8 | 393.2 | 377.4 | 0 | 0 | 0 | 0 |
| slot_encoders_real | purchased robot + optional LM393 slot encoders (encoder_mode 1), realistic | 101 | 99 | 99 | 98.0 | 87.1 | 3.53 | 68.1 / 117.4 | 7.7 | 402.8 | 390.0 | 25.0 | 1 | 0 | 0 | 0 |
| slot_encoders_harsh | purchased robot + optional slot encoders, harsh | 101 | 94 | 91 | 88.1 | 78.2 | 3.47 | 69.5 / 124.7 | 8.5 | 375.1 | 211.9 | 0.0 | 19 | 0 | 10 | 4 |

Runs in `final_real` that did not drive the true shortest path:

* gen_seed552: route stored correctly but the actual run left it (restart or wrong turn).
* gen_seed579: stored route 4057 mm vs true 3200 mm: the dry run ended at the box before the shorter branch was explored (strategy limit).
* gen_seed587: stored route 3443 mm vs true 3080 mm: the dry run ended at the box before the shorter branch was explored (strategy limit).
* gen_seed589: stored route 5381 mm vs true 4510 mm: the dry run ended at the box before the shorter branch was explored (strategy limit).
* gen_seed590: dry run not completed (2 restarts, faults: none).

Earlier results for the original N20 design (quadrature encoders, analog 8-channel array, odometer-based shortest-path referee) are archived in `results/benchmark_results_n20_v1.0.csv`.

## Exploration policy study (original N20 robot; tuning set: seeds 100–179 + sample, realistic)

| policy | profile | runs | dry % | actual % | shortest % | proven % | checkpoints | dry s (mean) | dry s (worst) | actual s (mean) | score (mean) | score (p10) | score (min) | restarts | false goals | line lost | faults |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dfs_LSR | real | 81 | 100.0 | 100.0 | 33.3 | 18.5 | 2.4 | 27.7 | 72.5 | 10.6 | 304.7 | 219.2 | 193.8 | 0 | 0 | 0 | 0 |
| dfs_SLR | real | 81 | 100.0 | 100.0 | 71.6 | 50.6 | 3.1 | 36.5 | 70.8 | 7.9 | 381.2 | 250.2 | 219.5 | 0 | 0 | 0 | 0 |
| dfs_SRL | real | 81 | 100.0 | 100.0 | 69.1 | 42.0 | 3.2 | 38.2 | 73.6 | 7.8 | 375.9 | 244.3 | 219.7 | 0 | 0 | 0 | 0 |
| dfs_RSL | real | 81 | 100.0 | 100.0 | 37.0 | 22.2 | 2.4 | 28.4 | 74.9 | 10.5 | 310.5 | 218.4 | 183.1 | 0 | 0 | 0 | 0 |
| left_hand | real | 81 | 100.0 | 100.0 | 33.3 | 19.8 | 2.4 | 30.7 | 102.0 | 10.6 | 302.1 | 219.2 | 193.8 | 0 | 0 | 0 | 0 |
| uc0.5_SLR | real | 81 | 100.0 | 100.0 | 77.8 | 56.8 | 3.1 | 37.2 | 75.5 | 7.7 | 391.2 | 250.2 | 226.5 | 0 | 0 | 0 | 0 |
| uc1_SLR | real | 81 | 100.0 | 100.0 | 77.8 | 58.0 | 3.3 | 41.5 | 85.3 | 7.7 | 390.6 | 253.6 | 226.5 | 0 | 0 | 0 | 0 |
| uc2_SLR | real | 81 | 100.0 | 100.0 | 97.5 | 87.7 | 3.6 | 61.8 | 102.8 | 7.2 | 411.8 | 396.6 | 242.8 | 0 | 0 | 0 | 0 |
| uc4_SLR | real | 81 | 100.0 | 100.0 | 97.5 | 91.4 | 3.6 | 72.5 | 113.6 | 7.2 | 402.1 | 378.8 | 226.1 | 0 | 0 | 0 | 0 |
| uc1_LSR | real | 81 | 100.0 | 100.0 | 50.6 | 30.9 | 2.8 | 35.0 | 91.4 | 9.5 | 337.3 | 221.7 | 193.8 | 0 | 0 | 0 | 0 |
| defer_SLR | real | 81 | 100.0 | 100.0 | 100.0 | 100.0 | 3.7 | 55.1 | 86.7 | 7.2 | 424.1 | 406.9 | 397.7 | 0 | 0 | 0 | 0 |
| defer_uc1_SLR | real | 81 | 100.0 | 100.0 | 100.0 | 100.0 | 3.7 | 56.6 | 90.7 | 7.2 | 423.6 | 406.1 | 393.8 | 0 | 0 | 0 | 0 |

Interpretation: `docs/exploration_strategy.md`.

## Problems found and fixed through testing

See `CHANGELOG.md` (1.1.0 for the purchased hardware: turn capture with a 15 mm digital pitch, phantom side branches, motor-imbalance trim, no-encoder stall detection, restart re-joining).

## Remaining weaknesses (known, not solved in software)

1. No wheel encoders: distances come from a speed model, so they are only as good as the Phase B calibration and the battery voltage. The harsh profile (10 % model error, unequal motors) is where most failures happen; see `slot_encoders_real` for what measured distance buys.
2. No battery sensing by default (no resistors): the firmware assumes a full pack. Charge before every match or fit the 47k/10k divider (`MESH_VBAT_SENSE=1`).
3. In the harsh profile a few runs fail calibration because one simulated comparator threshold never sees the dull line (the equivalent of a badly set pot). The robot refuses to start and names the channel; set every pot over the real floor.
4. The dry run ends at the first box arrival, so some mazes miss the shortest route (strategy limit; peek-and-prove needs the organisers' OK: Q4).
5. Sensor model: no ESP32 ADC non-linearity, no seams or glare on the vinyl, FC-51 modelled like a simple down-looking reflectance sensor, no lateral skid, gyro without vibration noise. Re-tune on the practice arena.
6. Not measured yet: BO motor rating and stall current, real sensor positions, the ESP32 board regulator.
