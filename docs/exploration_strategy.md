# Exploration strategy: chosen by expected score

## The scoring trade-off
| Term | Worth | What drives it |
|---|---|---|
| A: checkpoints | 25 each (sample: 4 = 100) | how much of the maze the dry run covers |
| B: dry run complete | 30 | reaching the box at all |
| C: 180 − dry time | 1 point per second saved | short dry run |
| S + T: shortest path | 30 + (150 − actual time) ≈ **170** | the dry-run map must contain the true shortest route |
| L: LED | 5 | box detection |

The decisive fact: **missing the shortest route costs about 170 points, while each extra second of
dry run costs 1 point.** Spending even 60 more seconds exploring is worth it if it raises the
probability of knowing the shortest route by more than about 35 percentage points. The dry run ends
when the robot reaches the box (the safe reading of the rules), so the *order* of exploration
decides what is known at that moment.

## Policies compared (closed-loop simulation)
Setup: 81 arenas (80 random mazes plus the reconstructed sample), realistic imperfection profile,
full matches with the firmware core in the loop: calibration → dry run → power cycle → actual run.
Data: `results/policy_study.csv`. Table: `results/policy_study_table.md`.
These are **simulation** results on *generated* mazes. In them START and the box sit at far-apart
leaves of a randomised grid; the real arena's structure may differ.

| policy | shortest % | proven % | checkpoints | dry s mean / worst | actual s | **score mean** | score p10 | score min |
|---|---|---|---|---|---|---|---|---|
| DFS, left-first (LSR) | 33.3 | 18.5 | 2.4 | 27.7 / 72.5 | 10.6 | 304.7 | 219.2 | 193.8 |
| DFS, right-first (RSL) | 37.0 | 22.2 | 2.4 | 28.4 / 74.9 | 10.5 | 310.5 | 218.4 | 183.1 |
| left-hand wall follower | 33.3 | 19.8 | 2.4 | 30.7 / 102.0 | 10.6 | 302.1 | 219.2 | 193.8 |
| DFS, straight-first (SLR) | 71.6 | 50.6 | 3.1 | 36.5 / 70.8 | 7.9 | 381.2 | 250.2 | 219.5 |
| DFS, straight-first (SRL) | 69.1 | 42.0 | 3.2 | 38.2 / 73.6 | 7.8 | 375.9 | 244.3 | 219.7 |
| SLR + START weight 0.5 | 77.8 | 56.8 | 3.1 | 37.2 / 75.5 | 7.7 | 391.2 | 250.2 | 226.5 |
| SLR + START weight 1 | 77.8 | 58.0 | 3.3 | 41.5 / 85.3 | 7.7 | 390.6 | 253.6 | 226.5 |
| **SLR + START weight 2 (default)** | **97.5** | 87.7 | 3.6 | 61.8 / 102.8 | 7.2 | **411.8** | **396.6** | 242.8 |
| SLR + START weight 4 | 97.5 | 91.4 | 3.6 | 72.5 / 113.6 | 7.2 | 402.1 | 378.8 | 226.1 |
| LSR + START weight 1 | 50.6 | 30.9 | 2.8 | 35.0 / 91.4 | 9.5 | 337.3 | 221.7 | 193.8 |
| peek-and-prove (SLR) ⚠ | 100.0 | 100.0 | 3.7 | 55.1 / 86.7 | 7.2 | 424.1 | 406.9 | 397.7 |
| peek-and-prove + weight 1 ⚠ | 100.0 | 100.0 | 3.7 | 56.6 / 90.7 | 7.2 | 423.6 | 406.1 | 393.8 |

All policies completed **100 %** of dry and actual runs, with no faults, restarts or false goals.

### What the numbers say
1. **Straight-first beats turn-first** (+70 points mean). Going straight both covers distance fastest
   and tends to follow the long axis towards a far goal. The routes found also have fewer turns, so
   the actual run is ~3 s faster.
2. **Biasing frontiers by distance from START** (uniform-cost flavour) is the biggest win among
   rule-safe policies. It finishes the branches near START before running deep, so the first route
   that reaches the box is usually the shortest. Weight 2 is the sweet spot: weight 4 is slower
   without finding more shortest routes. The *worst* case improves most: the 10th percentile rises
   from 250 to 397.
3. The price is a longer dry run (62 s mean vs 37 s, 103 s worst), which costs about 25 points of C, but it
   wins about 12 points of A (3.6 vs 3.1 checkpoints) and about 44 points of S+T on average (net +31). The worst dry
   run is still well inside 180 s.
4. **Peek-and-prove** (`defer_goal=1`): on first seeing the box the robot records it, backs out, and
   keeps exploring only until the Manhattan lower bound *proves* the known route shortest. It then
   returns and finishes. Best in every column, but the LED/box rule ("glow as soon as it senses a
   white box") and "dry run completed" timing make it **legal only if the organisers confirm it
   (Q4)**. It is implemented and tested, and switched off by default.

## Cautious vs. aggressive: when to pick which
| Situation | Setting |
|---|---|
| Default / unknown arena | `explore_pref=2`, `explore_start_weight=2` (most robust: p10 = 397) |
| Organisers allow seeing the box and leaving it again | `defer_goal=1` (+12 mean; 100 % shortest in simulation) |
| Robot is unreliable (frequent recoveries on the practice arena) | `explore_start_weight=0.5` (shorter dry run, less exposure to faults; 78 % shortest) |
| Arena turns out to be a tree (no loops) | any of these finds the unique route. Still use the default (checkpoint coverage) |

## Branch selection at a junction (exactly what the code does)
1. If following a planned path (backtracking or heading to a frontier), take its next exit.
2. Otherwise compute for every node with unexplored exits:
   `cost = time-weighted distance from here + explore_start_weight × distance from START`,
   where the time weighting adds 140 mm per 90° turn.
3. If the cheapest frontier is the current node, take its unexplored exit in **S > L > R** order.
   Otherwise drive there over known edges at `explore_known_speed_mm_s` (0.7 m/s) and continue.
4. If no frontier exists before the box was seen, the robot raises a NO_FRONTIER fault, because the
   box must have been missed. Check the box calibration.
