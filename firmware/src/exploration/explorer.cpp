#include "explorer.h"

#include "../util/util.h"

namespace mesh {

// Relative preference orders, REL_LEFT=3, REL_STRAIGHT=0, REL_RIGHT=1
static const uint8_t kPref[6][3] = {
    {REL_LEFT, REL_STRAIGHT, REL_RIGHT},   // 0 LSR
    {REL_LEFT, REL_RIGHT, REL_STRAIGHT},   // 1 LRS
    {REL_STRAIGHT, REL_LEFT, REL_RIGHT},   // 2 SLR
    {REL_STRAIGHT, REL_RIGHT, REL_LEFT},   // 3 SRL
    {REL_RIGHT, REL_LEFT, REL_STRAIGHT},   // 4 RLS
    {REL_RIGHT, REL_STRAIGHT, REL_LEFT},   // 5 RSL
};

const char* Explorer::prefName(int p) {
  static const char* names[6] = {"LSR", "LRS", "SLR", "SRL", "RLS", "RSL"};
  return (p >= 0 && p < 6) ? names[p] : "?";
}

uint8_t Explorer::pickUnexplored(const MazeGraph& g, int n, uint8_t h, uint8_t mask) const {
  (void)g;
  (void)n;
  int p = cfg_->explore_pref;
  if (p < 0 || p > 5) p = 0;
  for (int k = 0; k < 3; ++k) {
    uint8_t d = dirAdd(h, kPref[p][k]);
    if (mask & dirBit(d)) return d;
  }
  uint8_t back = dirOpp(h);
  if (mask & dirBit(back)) return back;
  return DIR_NONE;
}

uint8_t Explorer::decide(const MazeGraph& g, int n, uint8_t h) {
  // 1. Continue an active backtracking plan.
  if (plan_idx_ < plan_.n) {
    if (plan_.steps[plan_idx_].node == n) {
      return plan_.steps[plan_idx_++].exit;
    }
    Log::printf("EXP plan desync at node %d (expected %d), replanning", n, plan_.steps[plan_idx_].node);
    plan_.n = plan_idx_ = 0;
  }
  const Node& node = g.node(n);

  // 2. Wall-follower policies.
  if (cfg_->explore_policy == (int)ExplorePolicy::LEFT_HAND ||
      cfg_->explore_policy == (int)ExplorePolicy::RIGHT_HAND) {
    if (node.visits < 3) {
      static const uint8_t lh[4] = {REL_LEFT, REL_STRAIGHT, REL_RIGHT, REL_BACK};
      static const uint8_t rh[4] = {REL_RIGHT, REL_STRAIGHT, REL_LEFT, REL_BACK};
      const uint8_t* order = cfg_->explore_policy == (int)ExplorePolicy::LEFT_HAND ? lh : rh;
      for (int k = 0; k < 4; ++k) {
        uint8_t d = dirAdd(h, order[k]);
        if (node.exits & dirBit(d)) return d;
      }
      return DIR_NONE;
    }
    // fall through: loop detected -> frontier search
  }

  // 3. Uniform-cost bias: weigh each frontier by its distance from START so
  //    that branches near START are finished first (raises the chance that
  //    the first route found to the end box is the shortest one).
  uint8_t un = g.unexploredExits(n);
  if (cfg_->explore_start_weight > 0.0f) {
    int target = Planner::bestFrontier(g, n, h, 140.0f, cfg_->explore_start_weight, plan_);
    if (target < 0) {
      plan_.n = plan_idx_ = 0;
      return DIR_NONE;
    }
    if (target == n) {
      plan_.n = plan_idx_ = 0;
      return pickUnexplored(g, n, h, un);
    }
    plan_idx_ = 0;
    Log::printf("EXP heading for frontier node %d (%d steps, %.0f mm)", target, plan_.n, plan_.length);
    return plan_.n ? plan_.steps[plan_idx_++].exit : (uint8_t)DIR_NONE;
  }

  // 4. DFS: unexplored exit here?
  if (un) return pickUnexplored(g, n, h, un);

  // 5. Nearest frontier elsewhere.
  int target = Planner::nearestFrontier(g, n, h, 140.0f, plan_);
  if (target < 0) {
    plan_.n = plan_idx_ = 0;
    return DIR_NONE;
  }
  plan_idx_ = 0;
  Log::printf("EXP backtrack to node %d (%d steps, %.0f mm)", target, plan_.n, plan_.length);
  if (plan_.n == 0) return DIR_NONE;  // should not happen: frontier is here
  return plan_.steps[plan_idx_++].exit;
}

}  // namespace mesh
