# Meshmerize (Techfest 2026-27): rules extracted from the official PDF

**Source:** `Meshemerize_Problem_Statement_compressed.pdf`, 7 pages, read in full on 2026-10-24.
Wording is quoted where it matters. Page numbers refer to that PDF.
**The PDF is the authority.** Where this repository makes an assumption, it is labelled
**ASSUMPTION**, and the matching question for the organisers is in the last section.

## 1. Objective (p.1)
> "Design and build an autonomous robot capable of detecting and following a white line while tracking its direction of travel through a maze. The bot must first analyse the arena during a timed dry run, then use the path it has learned to navigate from the starting point to the ending point of the maze in the shortest possible time on its actual run."

## 2. Arena (p.1–3)
| Item | Official value | Where used |
|---|---|---|
| Arena size | 230 cm × 230 cm | `simulator/arena/arena.py` (`ARENA_SIZE`) |
| Lines | random paths of **white vinyl** strips on a dark field (figures show black) | sensor model, calibration |
| Line width | **30 mm** | `LINE_WIDTH`, branch threshold `branch_min_sensors` |
| Angle between adjacent lines | **90°** | discrete heading in the map (`robot_types.h`) |
| End zone | **400 mm × 400 mm white box** | `goal_detector.*`, `BOX_SIZE` |
| Tolerance | "accurate to within 5% or 20 mm, whichever is less" | match radius, relocalisation radius |
| Sample arena | Fig. 1 (top view, dimensions, checkpoints), Fig. 2 (isometric) | `simulator/arena/sample_arena.py` |
| Actual arena | "will consist of alterations in the path" | nothing is hard-coded; random-maze tests |

The dimension labels in Fig. 1 contradict each other. For example, the labels "48" and "28" near START
span almost equal distances. The sample arena was therefore reconstructed by pixel measurement
(±2 cm), not from the labels. See `sample_arena.py`.

## 3. Checkpoints (p.1–2, p.6)
* "The checkpoints are shown in the figure below. There will be no circles on the arena; they are used as references."
* "Each checkpoint carries 25 points." "25 points will be awarded if the bot crosses any of the checkpoints, but it will be counted only once for each checkpoint."
* Fig. 1 shows **4** checkpoints in the sample. All four sit on **junctions**: the top line at the stub,
  the right vertical, the left 4-way crossing, and the T above START.
* `A` counts checkpoints "covered during the Dry Run" only.

The robot cannot sense an invisible checkpoint, and it does not need to. The referee scores
crossings. The robot raises its checkpoint count by covering more of the maze during the dry run.
Checkpoint coordinates are **not** given to the robot. That would be feeding arena information, and
the real positions are unknown anyway.

## 4. Gameplay and timing (p.4)
1. **Dry run:** start at START and reach the end box. "The bot has to give a signal by glowing an LED as soon as it senses a white box below it/reach at end point." The robot may "store the turns in its memory". "There are no restrictions to cover all the checkpoints."
2. **Actual run:** "start from the 'start point' and end on the white box in min time by using the path stored in the bot after the first run."
3. Dry run: **3 min**. Actual run: **2 min 30 s**.
4. Total: **5 min 30 s**. "If a team fails to complete a dry run in 3 min, extra time will be deducted from the time period of the actual run."

## 5. Restarts (p.4)
1. Maximum **3 restarts in the entire match** (dry + actual combined).
2. A dry-run restart "has to start from the checkpoint".
3. "Once Dry Run is completed by the bot, the team won't be given any other chance for Dry Run."
4. An actual-run restart starts "from the start zone".
5. "The timer will not be set back to zero and will not be paused in any case."
6. "During a restart, a contestant must not feed information about the arena to the bot." Sensors may be adjusted and hardware changed.
7. No change may reduce the robot's weight (e.g. removing a sensor array, or fitting lighter batteries).

## 6. Robot specifications (p.5)
| Rule | Requirement | Design response |
|---|---|---|
| Size | fits in a 220 × 220 × 220 mm box | design envelope 150 × 140 × ~75 mm (`docs/mechanical_design.md`) |
| Start | "started by only one switch. However, a team may have an onboard switch." | one START push-button (all functions by press length) + one power switch |
| LED | a **red LED** that glows on reaching the end zone | GPIO15 red LED, lit when the box is confirmed (both runs) |
| Arena care | no damage, nothing left behind, no marks | rounded acetal skid / ball caster, no sharp edges, nothing detachable |
| Power | on-board supply | 4S 18650 pack |
| Voltage | potential difference between any two points ≤ **24 V** | max 16.8 V (4S, full) |
| Construction | no ready-made Lego kits or ready-made mechanisms; "can make use of ready-made gear assemblies" | BO gear motors are ready-made gear assemblies; the purchased 2WD chassis plate needs an organiser answer |

## 7. Game rules and general rules (p.5–6)
* **1 minute for calibration.** Changing code after the robots are deposited means disqualification. Hardware changes are allowed.
* One robot per team, autonomous. Nobody may touch the robot or enter the arena once it has started.
* Only one team member near the field when starting. The run starts on the organisers' signal.
* The start must not involve pushing or giving the robot any impulse.
* Laptops and PCs are not allowed near the arena. Wi-Fi, Bluetooth and similar devices must be switched off.
* The organisers' timing is final. Teams may not photograph or record their run.
* Team size: up to **4**. Teams may mix institutes.

## 8. Scoring (p.6–7), implemented in `simulator/scoring/score.py`
```
A = 25 × (checkpoints covered during the Dry Run)
B = 30 if the bot successfully completes the Dry Run
C = 180 − total time (s) to complete the Dry Run
S = 30 if the bot successfully completes through the Shortest Path
T = 150 − total time to complete the Actual Run (only if completed in the shortest path)
L = 5 if the LED glows
P = penalties
Total = (A + B + C + S + L + T) − P
```
Top 5 teams per zonal round go to the Grand Finale.

## 9. Ambiguities and questions for the organisers
Each item lists the default used in this repository; the simulator makes it configurable.

| # | Question to ask | Why it matters | Default here |
|---|---|---|---|
| Q1 | What exactly counts as "crossing" a checkpoint? Any part of the robot over the reference point, or its centre? | Checkpoint score; restart position | Axle centre within 60 mm (`MatchConfig.checkpoint_radius_mm`) |
| Q2 | Are checkpoints always at junctions, as in the sample? How many will the real arena have? | Restart placement, relocalisation | Junctions; any number |
| Q3 | A dry-run restart "has to start from the checkpoint": the last one crossed, or the nearest? If none was crossed, from START? Which way will the robot face? | The robot must relocalise | Last crossed (START if none), facing the way the robot left it |
| Q4 | Does the dry run end the moment any part of the robot enters the box, or when the LED glows? May the robot see the box, back out, keep exploring, and return later? | Decides whether "peek-and-prove" exploration (`defer_goal=1`) is legal; this can raise the shortest-path rate | **Not legal** (`defer_goal=0`) until confirmed |
| Q5 | How is "shortest path" judged: by distance, by the referee's chosen route, and with what tolerance? Are several equal-length routes all accepted? | S and T | By the route driven (sequence of junctions), any equally short route |
| Q6 | What is C if the dry run takes longer than 180 s: negative, zero, or not completed? | Score maths | Literal (negative allowed); `clamp_negative` option |
| Q7 | What penalties make up P, and how much is each worth? | Score | 0 (none are defined in the PDF) |
| Q8 | Is the time between the dry and actual runs (carrying the robot back to START) counted in the 5 min 30 s? | Timing | Not counted |
| Q9 | Between runs, may the operator switch the robot off and on, or press a button to tell it the next run is the actual run? | Stage handling | Robot decides by itself from stored flash data. Only the start button is pressed. Power cycling is supported but not needed. |
| Q10 | Is L awarded per run, or once? | Score | Once |
| Q11 | Will START and the END box stay where the sample shows them (bottom right and bottom left)? | A goal-direction prior would be legal only if published | No prior used |
| Q12 | Are the field and line materials as in the figures (matte black field, white vinyl)? How glossy is the end box surface? | Sensor thresholds | Configurable; calibrated on site |
| Q13 | Does the 1-minute calibration allow driving the robot on the arena line (sweep calibration)? | Calibration method | Yes, a 2-second in-place sweep at START |
| Q14 | Does "only one switch" ban a separate power switch? | Electrical design | The PDF allows an "onboard switch". One power switch plus one start button. |
| Q15 | Is a purchased 2-wheel chassis plate (with motor brackets and caster) a "ready-made mechanism"? | Mechanical build | Ask; fallback is a self-made plate of the same size |
| Q16 | Is an ESP32 allowed if its Wi-Fi and Bluetooth are never switched on (the module has radios)? | Controller | Ask; the firmware never starts the radios and calls `btStop()` |
