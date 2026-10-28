// planner.h - shortest routes on the discovered graph.
//
// Dijkstra over (node, arrival heading) states so that the cost of turning
// at a node can be included:
//     cost = sum(edge length) + turn_weight * (number of 90-degree turns)
// For the competition route turn_weight is small (config turn_tiebreak_mm,
// default 30 mm): physical distance dominates, and among routes of equal
// measured length (within a few cm of odometry noise) the one with fewer
// turns - which is also the faster one - wins.
// For exploration the planner is run with a larger turn weight so that
// "nearest frontier" approximates nearest in TIME.
#pragma once
#include "../maze/graph.h"
#include "config.h"

namespace mesh {

struct RouteStep {
  uint8_t node;   // node where the action happens
  uint8_t exit;   // absolute exit direction taken at that node
  int16_t edge;   // edge followed after leaving (-1 = unexplored line)
};

struct Route {
  int n = 0;
  RouteStep steps[kMaxRouteSteps];
  float length = 0;     // mm, sum of edge lengths
  int turns = 0;        // node turns + corners
  uint8_t start_heading = DIR_N;
  bool valid = false;
};

class Planner {
 public:
  static bool shortest(const MazeGraph& g, int start, uint8_t heading, int goal, float turn_weight, Route& out);
  // Route to the cheapest node that still has unexplored exits (the final
  // unexplored step is NOT included). Returns the node index or -1.
  static int nearestFrontier(const MazeGraph& g, int start, uint8_t heading, float turn_weight, Route& out);
  // Lower bound on any START->GOAL route that uses at least one unexplored
  // line: min over frontier nodes f of dist(START,f) + |f - GOAL|_1.
  // (Valid because every line is axis-aligned.)
  static float unexploredLowerBound(const MazeGraph& g, int start, uint8_t heading, int goal);
  static int turnCount(uint8_t heading, uint8_t exit);  // 0, 1 or 2 quarter turns
  // Cheapest cost (over arrival headings) from `start` to every node.
  static void distancesFrom(const MazeGraph& g, int start, uint8_t heading, float turn_weight, float* out);
  // Frontier selection with a START-distance weight (uniform-cost bias):
  // minimises dist(robot,f) + start_weight * dist(START,f).
  static int bestFrontier(const MazeGraph& g, int robot, uint8_t heading, float turn_weight, float start_weight,
                          Route& out);

 private:
  static void dijkstra(const MazeGraph& g, int start, uint8_t heading, float turn_weight);
};

}  // namespace mesh
