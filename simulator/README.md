# Meshmerize simulator

A white-line maze simulator written for this competition. It is not a converted wall-maze
simulator. The robot under test is the **real firmware core** (`firmware/src`, compiled for the PC
by `tools/build_core.py` and loaded with ctypes). The simulator supplies only what the hardware
would: raw reflectance ADC values, gyro rate, encoder counts (if fitted), battery voltage, button presses.
**The robot never sees the true maze**: no node positions, no checkpoints, no goal.

## Install and run
```bash
pip install -r requirements.txt          # pygame, numpy, pytest, ziglang (C++ compiler), matplotlib
python -m simulator.main                  # GUI on the reconstructed sample arena
python -m simulator.main --arena gen:12 --profile harsh --speed 8
python -m simulator.main --headless --out results/run.json
python -m simulator.main --inject dry:15:press:0      # judge-called restart at t=15 s
python -m pytest simulator/tests -q                   # unit + closed-loop tests
python tools/run_batch_tests.py --mazes 40 --label mytest --set explore_start_weight=2
```
GUI keys: SPACE pause, + / − speed, M robot map on/off, T trail, Q quit.

## Contents
| Module | Models |
|---|---|
| `arena/arena.py` | 230 × 230 cm arena, 30 mm lines (axis-aligned), 400 × 400 mm box, START, checkpoints, JSON I/O. `TruthGraph`: an **independent** Python Dijkstra for the true shortest path (cross-checks the C++ planner) |
| `arena/sample_arena.py` | rulebook Figure 1, reconstructed by pixel measurement (±2 cm) |
| `arena/generator.py` | random rule-compliant mazes: loops, dead ends, T, crossings; box at a leaf; checkpoints on junctions |
| `sensors/reflectance.py` | IR array + wing sensors: optical spot coverage, per-channel gain/offset, ADC noise, analog (QTR) or digital comparator outputs with per-channel threshold and hysteresis (TCRT5000/FC-51), dull vinyl, shiny floor, box reflectance, dead channel, sample delay |
| `robot/physics.py` | DC motor first-order dynamics, battery sag and drain, wheel-diameter error, motor gain mismatch, random and acceleration slip, quadrature or slot encoders (or none), MPU6050 gyro (bias, noise, scale error), polarity faults, jammed wheels |
| `robot/sim_robot.py` | couples physics, sensors and the core. Mirrored right motor, as on the real chassis |
| `robot/core_bridge.py` | ctypes bindings (robot + direct graph/planner/persistence test API) |
| `scoring/score.py` | the official formula A+B+C+S+L+T−P and the clock rules |
| `scoring/match.py` | full match: calibration minute, dry run, power cycle, actual run, referee (checkpoint crossing, restarts at the last checkpoint, max 3, clocks never paused, off-track/stuck detection, shortest-path judgement), fault injection |
| `visualization/viewer.py` | pygame: arena, robot, sensor spots, the robot's own map and route, clocks, score, log |
| `config.py` | ideal / realistic / harsh imperfection profiles. Defaults = the purchased robot (BO motors without encoders, 5-ch digital array + 2 wings, MPU6050, 4S pack without voltage sensing); `n20_hardware()` = the original design |

## Known differences from the real robot
* Lines are perfect rectangles. Real vinyl has seams, slight curves, scuffs and glare.
* No lateral wheel skid, no caster drag, no gearbox backlash, a perfectly flat floor.
* Motor model: first-order, linear in voltage. Real BO motors vary unit to unit (more than N20s), and their speed depends
  on temperature and load.
* Sensor model: linear spot coverage plus noise. The ESP32 ADC's non-linearity and channel
  crosstalk are **not** modelled (calibration absorbs the former; the firmware double-read
  mitigates the latter).
* Bumps are instantaneous (kick/drag injection). Real bumps are smoother.
* The referee's interpretations (checkpoint radius, restart placement, shortest-path tolerance) are
  assumptions; see `docs/competition_rules.md`.

Simulation results are evidence about the **algorithms**. They are not a substitute for Phases A–G
of `docs/calibration_guide.md` on the real robot.
