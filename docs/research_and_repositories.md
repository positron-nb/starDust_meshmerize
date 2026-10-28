# Research report: reference repositories and sources

**What was actually done.** Each repository below was shallow-cloned with `git clone --depth 1` on
2026-10-24. The source files listed were opened and read. Commit hashes are the HEAD of that
clone. **Nothing was compiled, run, or tested from these repositories.**
The licence column reflects the files in the clone, not legal advice.
Where a repository has no licence file, its code is "all rights reserved" by default. This
project took **ideas only** from those, and wrote everything new.

## Wall mazes (Micromouse) vs. white-line mazes
| | Micromouse (walls) | Meshmerize (white lines) |
|---|---|---|
| World | square cells (18 cm), walls on cell edges | arbitrary 30 mm lines, 90° junctions, any spacing |
| Sensing | distance sensors see walls of the *current and next* cell | reflectance array sees the line **under** the robot only |
| Localisation | cell grid is known in advance: position = cell index | no grid: position comes from odometry plus junction matching |
| Algorithm | flood fill on a known-size grid toward a known goal (the centre) | goal position unknown until reached; graph exploration (DFS/Trémaux), then Dijkstra |
| Re-planning | robot can run back and forth; flood fill updates cell values | the dry run ends at the goal, so exploration order decides what is known |

A flood-fill implementation built for walls (repos 3, 4, 6) cannot be reused unchanged. It assumes
a known grid, a known goal cell, and wall observations of neighbouring cells, and none of these
exist here. What carries over is the *idea* of running a shortest-path search over a growing map.

---

### 1. Abdelaal495/Maze-Solving-Robot
* **URL:** https://github.com/Abdelaal495/Maze-Solving-Robot (commit `765b9b0`, 2023-06-05)
* **Licence:** none in the repository, so no adaptation is permitted (ideas only).
* **Stack:** Arduino (Nano/Uno/Mega), 5 × TCRT5000 digital sensors, 2 × L298N, 4 DC motors, no encoders.
* **Files read:** `MazeSolver.ino`. Key functions: `readSensors`, `computePosition`, `moveStraight` (PID), `turnLeft/turnRight/turnAround`, `analyzeJunction`, `updatePath`, `simplifyDirections`, `done`, `solveMaze_SecondTrial`.
* **Algorithm:** left-first (or right-first) wall follower that records L/S/R/B. After the run it replaces `xBy` triples with a single turn (LBL→S, LBR→B, …) and then replays the reduced string.
* **Strengths:** the classic path-reduction idea in compact form; the second run is a direct replay.
* **Weaknesses found in the code:**
  * `delay(1000)` stops in every turn: blocking, slow, and the sensors are not read during these delays.
  * Digital sensors only.
  * Finish detection is a specific 1-0-1 pattern (`checkFinishLine`).
  * `simplifyDirections` has no return for the default case (undefined behaviour).
  * The reduction is only correct for **loop-free** mazes.
  * No encoders, so no distances and no shortest-*distance* route.
* **Usable here:** the path-reduction concept, used only as a sanity check. The final design uses a metric graph and Dijkstra instead.
* **Must be rewritten:** all of it.

### 2. sapertuz/mazeSolver (fork of cheiio/mazeSolver)
* **URL:** https://github.com/sapertuz/mazeSolver (commit `6962302`, 2020-09-01)
* **Licence:** `Maze_Solver/LICENSE.txt` is **CC-BY 3.0** (the 3D-printed parts, from Thingiverse 1690543). The README shows a licence badge for `cheiio/mazeSolver`, but there is no licence file at the root of the clone. Treat the code as unlicensed.
* **Stack:** Arduino Leonardo, 4 analog QTR channels plus 2 digital QRD side sensors, PID_v1 library. Eagle PCB, SolidWorks parts with STEP and STL.
* **Files read:** `Arduino_src/Maze.ino`, `calibrate_sensor.ino`, `follow_segment.ino`, `read_line.ino`.
* **Algorithm:** right-first decision at crossings. `calibrate_line_sensors()` spins the robot and records per-channel min/max, then derives timing constants from the spin time.
* **Strengths:** **sweep calibration** at the start (adopted here as the 1-minute calibration routine). Separate side sensors for junction detection. Printable chassis files.
* **Weaknesses:**
  * Timing is built on delays (`ts = 50 ms` PID period).
  * The `path[]` array is declared but no maze memory or second-run logic is implemented.
  * The finish is detected as "white space" plus timing.
* **Usable:** the calibration sweep and the side-sensor concept. A wider array covers both here.

### 3. Vatsal314/Micromouse
* **URL:** https://github.com/Vatsal314/Micromouse (commit `7ed673c`, 2026-07-13)
* **Licence:** none, so ideas only.
* **Stack:** **STM32F411CEU6 Black Pill**, VL53L0X ToF, MPU6050, 12 V 1000 RPM N20 encoder motors, **TB6612FNG**, 11.1 V LiPo, KiCad PCB.
* **Files read:** `Code Files/algorithm_code/algorithm_code.ino` (flood fill with a queue, `rearrange_map`, `direction_wrt_compass`), `Final_PID/Final_PID.ino`.
* **Defect found:** `initWalls()` line 71 reads `wall_data[r][c][3] = (r= length-1);`. That is an assignment, not a comparison. It marks south walls wrongly and overwrites the loop variable.
* **Strengths:** independent evidence that the Black Pill + TB6612FNG + N20-encoder combination works in a small student robot.
* **Not applicable:** wall-based flood fill, ToF wall sensing. A 3S battery is unnecessary here.

### 4. raghadjam/MicroMouse
* **URL:** https://github.com/raghadjam/MicroMouse (commit `c82e026`, 2026-04-03)
* **Licence:** none, so ideas only.
* **Stack:** ESP32, IR wall sensors, MPU6050 yaw, N20 encoders. 3D-printed chassis (`micromouse.stl`). 8 × 8 maze.
* **Files read:** `floodfill.cpp` (1068 lines): `floodFillToGoal`, `floodFillToStart`, `reconstructPath`, `updateTurnStep90`, encoder ISRs.
* **Weakness:** encoder ISRs only count up (`countA++`), so direction is lost and reversing corrupts odometry. This project uses hardware quadrature decoding instead.
* **Strengths:** explore → return → fast-run structure; turn PID with gyro.
* **Not applicable:** wall world. ESP32 radios must be switched off under the rules, so a non-radio MCU avoids any argument with the referees.

### 5. pololu/qtr-sensors-arduino
* **URL:** https://github.com/pololu/qtr-sensors-arduino (commit `4a0d1c5`, 2019-03-15)
* **Licence:** permissive MIT-style (`LICENSE.txt`: "Permission is hereby granted, free of charge…"). Adaptation is permitted with the copyright notice.
* **Files read:** `QTRSensors.cpp` (`calibrateOnOrOff`, `readCalibrated`, `readLinePrivate`, emitter control).
* **Ideas adopted (reimplemented, not copied):**
  * per-channel min/max calibration;
  * normalisation to 0–1000;
  * a weighted centroid with a noise floor;
  * remembering which side the line was last seen on when it is lost.
* **What differs here:**
  * No heap. `calibrateOnOrOff` uses `realloc`.
  * The centroid is taken over the **contiguous white run nearest the previous position**. The Pololu centroid averages every lit channel, so a side branch drags it.
  * Branch and goal features are extracted for junction classification.

### 6. zrhnd/simulator-micromouse
* **URL:** https://github.com/zrhnd/simulator-micromouse (commit `f6d2e28`, 2026-10-18)
* **Licence:** no root licence. The bundled `mms-cpp` template is MIT (© Mack Ward).
* **Stack:** STM32F401 target. The mms simulator, `flood-fill-c` (BFS distance map), `astar-c`, a Q-learning experiment, a Webots world. 522 competition maze files.
* **Files read:** `README.md`, the directory structure, the `mms-cpp/LICENSE` header.
* **Idea adopted:** keep the algorithm behind a small API so that the same code runs in the simulator and on the MCU. This project goes further: the *entire* firmware core is compiled into the simulator (`tools/build_core.py`).
* **Not applicable:** a cell/wall world. No reflectance or physics model.

### 7. (additional) pololu/libpololu-avr: 3pi maze solver
* **URL:** https://github.com/pololu/libpololu-avr (commit `718e109`, 2022-10-26), `examples_templates/3pi-mazesolver/`
* **Licence:** Pololu `LICENSE.txt` (permissive). Only the header was read; check the full text before copying anything.
* **Files read:** `maze-solve.c` (`select_turn`, `simplify_path`, `maze_solve`), `follow-segment.c`.
* **Algorithm:** left-hand rule. `simplify_path` sums the angles of the last three moves when the middle one is `B`. Intersections are confirmed by driving 50 ms, checking the outer sensors, driving 200 ms more, then checking the middle sensors.
* **Key lesson:** its finish test is `sensors[1..3] > 600` after an intersection. That is exactly the "all white = finish" shortcut that fires on a large white junction. This project instead requires all-white to persist for **70 mm of travel** and then confirms it with a 140 mm drive (`goal_detector.*`).

### 8. Literature found by web search (not studied in depth)
* Westminster/arXiv paper *1410.4145* (found via search; abstract-level information only): coordinate-based maze mapping, an adjacency-list graph, and Dijkstra for the shortest and fastest paths. Its approach matches the one chosen here.
* Forum threads (Parallax, Arduino) make the same points: treat junctions as nodes, and wall followers fail on loops.

## Component data used (verified on manufacturer pages, 2026-10-24)
| Part | Fact | Source |
|---|---|---|
| Pololu QTR-MD-08A | 8 ch, 8 mm pitch, 2.9–5.5 V, analog 0–VCC, 30 mA per LED pair, 125 mA max, 61 × 20 × 2.5 mm, 3.1 g, US$9.90 | https://www.pololu.com/product/4248 |
| Pololu micro metal gearmotor table | 6 V stall currents: LP 0.36 A, MP 0.67 A, HP 1.6 A, HPCB 1.5 A. 6 V HPCB 50:1 = 650 RPM, HP 50:1 = 590 RPM | https://www.pololu.com/category/60/micro-metal-gearmotors |
| Pololu magnetic encoder | 12 CPR at the motor shaft (both edges, both channels), 2.7–18 V, 10 kΩ pull-ups | https://www.pololu.com/product/3081 |
| TB6612FNG carrier | VM 4.5–13.5 V, 1 A continuous per channel, **3 A peak**, PWM ≤ 100 kHz, STBY pin | https://www.pololu.com/product/713 |
| DRV8833 carrier | VM 2.7–10.8 V, ~1.2 A continuous, 2 A peak | https://www.pololu.com/product/2130 |
| Black Pill F411 | ₹499 incl. GST (Probots, in stock at the time of search) | https://probots.co.in/stm32f411ceu6-development-board-arm-cortex-m4-32-bit-microcontroller.html |
| TB6612FNG module | ₹209 incl. GST (Probots) | https://probots.co.in/tb6612fng-dual-channel-dc-motor-driver-module.html |
| Orange 2S 1000 mAh 20C LiPo | ₹899 incl. GST (Robu; stock varies) | https://st.robu.in/product/orange-1000mah-2s-20c-7-4v-lithium-polymer-battery-pack-lipo-2 |
