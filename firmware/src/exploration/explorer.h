// explorer.h - branch selection during the dry run.
//
// DFS_NEAREST_FRONTIER (default)
//   * At a node with unexplored exits, take one of them in the configured
//     relative preference order (explore_pref, e.g. L > S > R).
//   * Otherwise drive (over known edges, at the faster "known edge" speed)
//     to the frontier node that is cheapest to reach - measured as length +
//     ~turn time - and continue there. This is Tremaux / DFS generalised to
//     a metric map: it never re-enters a finished branch and handles loops.
//   * If no frontier is left the maze is fully explored.
// LEFT_HAND / RIGHT_HAND
//   * Classic wall-follower order. Wall following can circle forever around
//     an island loop that does not contain the goal, so once a node has been
//     visited 3 times the explorer falls back to DFS_NEAREST_FRONTIER.
#pragma once
#include "../maze/graph.h"
#include "../planner/planner.h"
#include "config.h"

namespace mesh {

class Explorer {
 public:
  void init(const RobotConfig* cfg) { cfg_ = cfg; reset(); }
  void reset() { plan_.n = 0; plan_idx_ = 0; }
  // Robot stands on node n having arrived with heading h. Returns the
  // absolute exit to take, or DIR_NONE when nothing is left to explore.
  uint8_t decide(const MazeGraph& g, int n, uint8_t h);
  bool backtracking() const { return plan_idx_ < plan_.n; }
  // Follow an externally planned route (e.g. "go to the goal now").
  void setPlan(const Route& r) { plan_ = r; plan_idx_ = 0; }
  static const char* prefName(int pref);

 private:
  uint8_t pickUnexplored(const MazeGraph& g, int n, uint8_t h, uint8_t mask) const;
  const RobotConfig* cfg_ = nullptr;
  Route plan_;
  int plan_idx_ = 0;
};

}  // namespace mesh
