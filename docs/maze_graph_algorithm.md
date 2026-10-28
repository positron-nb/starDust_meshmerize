# Maze mapping: from reflectance samples to a graph

Code: `sensors/line_sensor.*`, `maze/junction_detector.*`, `maze/graph.*`, `app/robot.cpp`
(`onEvent`, `onCorner`, `onNodeArrival`, `mapArrival`), `exploration/relocalizer.*`.

## 1. Junction recognition (`junction_detector.cpp`)
* **Features** for each 1 ms sample:
  * the calibrated 0–1000 values;
  * the **main white run** (the contiguous run nearest the previous line position);
  * the line position (centroid of that run);
  * `left_branch` / `right_branch`: the main run touches an array edge **and** is at least
    `branch_min_sensors` = 5 channels wide. A 30 mm line can cover at most 4 channels at 8 mm pitch;
  * `all_white` (every channel) and `goal_white` (all but one).
* **TRACK → CROSSING:** a branch pattern on 2 consecutive samples. Steering then **freezes** to an
  heading hold (gyro, or encoders when fitted), so the line follower is not pulled sideways by the cross line.
* **CROSSING → LOOKAHEAD:** the branch pattern has been absent for 4 mm. The crossbar spans
  s0…s1 (odometry). Left and right flags are OR-ed over the whole crossing.
* **LOOKAHEAD (16 mm):** majority vote on "line under the centre", which gives the straight exit.
* **Event position:** the array is L<sub>s</sub> = 70 mm ahead of the axle, so the junction centre
  is under the axle at `s_center = (s0+s1)/2 + 70`. The robot has about 50 mm left to stop for a
  pivot turn.
* **Classification:**
  * one side only and no straight: a **corner** (inside an edge, not a node);
  * ≥ 2 exits: a **junction node**;
  * line gone while it was centred: a **dead end**;
  * line gone while it was at the array edge: **LINE_LOST**, which starts recovery;
  * 70 mm of white: **GOAL**.
* **Measured (simulation):** with the tuned PD gains, the line offset at junction entry is 1.8 mm
  RMS and at most 4–5 mm. "Both sides" requires *all 8* channels, so a one-sided branch read at an
  8 mm offset is not mistaken for a crossing.

## 2. Graph representation (`graph.h`)
| Node field | Meaning |
|---|---|
| `x, y` | odometry position estimate (mm, map frame: START at the origin facing N) |
| `exits` | 4-bit set of absolute directions with a line |
| `edge[4]` | edge index per exit, −1 = unexplored |
| `type` | JUNCTION, DEAD_END, START, GOAL |
| `visits` | how often the robot stood on it |
| `odo` | odometry reading at the last visit (drives the match radius) |

| Edge field | Meaning |
|---|---|
| `a, da, b, db` | endpoints and the exit used at each end |
| `length` | centreline length (running mean over traversals) |
| `seg_len[], seg_turn[]` | straight segments and the ±90° corners between them |
| `traversals` | how many times it has been driven |

Validation (`MazeGraph::validate`) checks link symmetry and heading arithmetic. Leaving `a` in `da`
and applying the corner turns must arrive moving opposite to `db`. It also checks for positive
lengths.

## 3. New node vs. repeat visit (`MazeGraph::matchNode`, `Robot::mapArrival`)
A known node *m* is accepted as the current junction only if all of these hold:
1. its distance from the odometry estimate is ≤ r<sub>m</sub> = min(150, 70 + 0.03 · distance
   driven since the robot was last on *m*);
2. it has a line in the arrival direction;
3. its exit set matches. A one-branch mismatch is accepted only within r<sub>m</sub>/2;
4. its line in the arrival direction is unexplored **or** leads back to the node we came from (topology);
5. it is not a self-loop with fewer than 3 corners (geometrically impossible);
6. no second candidate is nearly as close (**ambiguity guard**: duplicates are safe, wrong merges are not).

A match is a **loop closure**: the pose snaps to the node, which removes accumulated drift.
Otherwise a new node is created.

## 4. Dead ends, backtracking, loops
* A dead end becomes a node. The robot brakes, makes a 180° pivot with line capture, and keeps the
  stopping offset in the next edge's distance bookkeeping.
* **Backtracking** uses the planner: drive over known edges at the faster "known-edge" speed to
  the cheapest frontier. Finished branches are never re-entered. A self-loop or a revisited node
  becomes an edge, not an endless cycle.
* **Graph completion:** no node has an unexplored exit, so `hasFrontier()` is false. The dry run
  normally ends earlier, at the box.

## 5. False detections and consistency
* **Phantom exit** (a branch that does not exist): the turn into it finds no line within the
  overshoot limit, so the exit is removed, the robot turns back to its arrival heading and decides again.
* **Exit-set mismatch at a known node:** logged. The union is kept during the dry run.
* **Unexpected feature on a known edge** (more than 60 mm + 15 % before the expected corner/node): ignored.
* **Map inconsistency** (≥ 2 topology conflicts since the last verified loop closure, e.g. the robot
  was bumped onto another line): the map is rolled back to the last verified state and the robot
  relocalises (see §6).

## 6. Relocalisation after a restart (`relocalizer.cpp`)
* The pre-restart map is kept, and a local map is built from the restart point (node 0).
* **Anchored mode.** Node 0 must coincide with an old junction, and the robot's heading must be one
  of that junction's exits. A candidate survives while every new local node lands on an old node
  with the same exits, or on unexplored ground.
* A single survivor is accepted. If several survive when the box is reached, the one with the most
  matches wins, then the most recently visited junction (the referee uses the *last* checkpoint
  crossed).
* **Free mode** (fallback) needs two junction matches and a unique solution.
* After a merge, the stored route starts at START again, so the actual run is unaffected. If no
  merge was possible, the START-based map is kept and the actual run searches.

## 7. Limitations (honest)
* Positions come from wheel odometry only. Errors grow by roughly 1–3 % of the distance between loop
  closures. Junctions closer than about 2 × the match radius (~150–300 mm) in an area not revisited
  for a long time may be duplicated rather than merged. The route then still exists, but a shorter
  loop through them may be missed.
* Relocalisation assumes checkpoints are junctions, as in the sample (Q2 in `competition_rules.md`).
* Edges are treated as axis-aligned polylines. Curved lines are not part of the rules.
