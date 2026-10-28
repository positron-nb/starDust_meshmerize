# Shortest path and route execution

Code: `planner/planner.*`, `app/robot.cpp` (`ROUTE_PLANNING`, `beginEdge`, `planSpeed`, `decideExit`).

## Algorithm
Dijkstra over **(node, arrival heading)** states (≤ 96 × 4), so turning cost depends on how the
robot arrives:
```
cost(edge) = length_mm + w_turn × (corners inside the edge)
cost(node) = w_turn × quarter_turns(arrival heading → chosen exit)   (straight 0, left/right 1, U-turn 2)
```
* **Competition route:** w<sub>turn</sub> = `turn_tiebreak_mm` = **30 mm**. Distance dominates (the
  organisers judge the shortest *path*). Among routes whose measured lengths differ by less than
  ~30 mm per turn, which is within odometry noise, the one with fewer turns wins. It is also faster.
* **Exploration "nearest frontier":** w<sub>turn</sub> = 140 mm (about the time cost of a pivot at
  exploration speed), so the robot goes to the frontier that is nearest in *time*.
* The GOAL node is a sink: a route never passes through the box.
* Complexity: O(V²) with V ≤ 384 states, about 150 k operations. Under 1 ms on the ESP32.

## Route simplification
The route is a list of `(node, exit, edge)` steps taken directly from the graph, so it contains
only physical turns:
* **Dead ends and U-turns** never appear in a shortest route. A U-turn would always add length,
  so Dijkstra drops them automatically (the graph equivalent of LSRB `xBy` reduction).
* **Corners** are kept inside each edge and executed when the detector reports them. They are never
  removed, because they are required to follow the track.

## Certificate ("is it provably shortest?")
Every line is axis-aligned, so any path between two points is at least as long as their Manhattan
distance. When the box is reached:
```
LB = min over frontier nodes f of [ dist_known(START, f) + |f − GOAL|₁ ]
```
If LB ≥ known route − 20 mm, no unexplored line can give a shorter route. The log then prints
**PROVABLY SHORTEST**. This is also the stopping rule of the optional peek-and-prove mode.

## Executing the stored route (actual run)
* For each edge the oriented polyline is known. On each straight segment the robot accelerates to
  `actual_speed_mm_s` (1.0 m/s) and brakes so that it reaches `actual_approach_mm_s` (0.32 m/s)
  90 mm before the expected corner or junction. A junction the route goes **straight** through only
  slows to `actual_pass_speed_mm_s` (0.7 m/s).
* The **sensor event**, not the odometry estimate, triggers each turn. Odometry only shapes the
  speed profile. Accumulated distance error therefore cannot cause a missed turn.
* Each arrival is checked against the map. An early unexpected feature is ignored; a mismatch
  (e.g. a dead end) switches to "replan to goal from here".

## Unit tests (C++ planner through ctypes: `simulator/tests/test_planner_graph.py`)
* direct vs. detour route;
* loop with equal halves;
* equal length broken by fewer turns;
* length beats the turn tie-break;
* unreachable goal;
* never routing through the goal;
* lower-bound certificate;
* frontier selection;
* heading-consistency validation;
* persistence round trip and CRC rejection.
