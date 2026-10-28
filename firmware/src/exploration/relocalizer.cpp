#include "relocalizer.h"

#include <math.h>

#include "../util/util.h"

namespace mesh {

static void rot(int r, float x, float y, float& ox, float& oy) {
  // r quarter turns clockwise: (x, y) -> (y, -x)
  for (int i = 0; i < (r & 3); ++i) {
    float t = x;
    x = y;
    y = -t;
  }
  ox = x;
  oy = y;
}

uint8_t Relocalizer::rotMask(uint8_t m, int r) {
  uint8_t o = 0;
  for (int d = 0; d < 4; ++d)
    if (m & dirBit((uint8_t)d)) o |= dirBit(dirAdd((uint8_t)d, r));
  return o;
}

void Relocalizer::transform(const Alignment& a, float x, float y, float& ox, float& oy) {
  rot(a.rot, x, y, ox, oy);
  ox += a.tx;
  oy += a.ty;
}

static bool isJunction(const Node& n) { return n.type == (uint8_t)NodeType::JUNCTION; }

// Checks every local node (except node 0 and the goal) against the old map.
// Returns false on a conflict; counts nodes that coincide with an old node.
static bool consistent(const MazeGraph& old_map, const MazeGraph& local, const Alignment& c, float radius,
                       int& matched) {
  matched = 0;
  for (int i = 1; i < local.nodeCount(); ++i) {
    const Node& ln = local.node(i);
    if (ln.type == (uint8_t)NodeType::GOAL || ln.type == (uint8_t)NodeType::START) continue;
    float qx, qy;
    Relocalizer::transform(c, ln.x, ln.y, qx, qy);
    uint8_t ex = Relocalizer::rotMask(ln.exits, c.rot);
    int hit = -1;
    float hd = radius;
    for (int k = 0; k < old_map.nodeCount(); ++k) {
      const Node& kn = old_map.node(k);
      if (kn.type == (uint8_t)NodeType::GOAL) continue;
      float d = sqrtf((kn.x - qx) * (kn.x - qx) + (kn.y - qy) * (kn.y - qy));
      if (d < hd) { hd = d; hit = k; }
    }
    if (hit < 0) continue;  // unexplored ground in the old map: allowed
    const Node& kn = old_map.node(hit);
    // START is a line end: a local dead end may land on it
    bool same = kn.exits == ex || (kn.type == (uint8_t)NodeType::START && ln.type == (uint8_t)NodeType::DEAD_END &&
                                   (kn.exits & ex) == ex);
    if (!same) return false;
    ++matched;
  }
  return true;
}

int Relocalizer::alignAnchored(const MazeGraph& old_map, const MazeGraph& local, float radius, Alignment& best) {
  if (local.nodeCount() < 1) return 0;
  const uint8_t ex0 = local.node(0).exits;
  int count = 0;
  bool have = false;
  for (int o = 0; o < old_map.nodeCount(); ++o) {
    const Node& on = old_map.node(o);
    // restart points: a checkpoint junction, or START when no checkpoint was crossed yet
    if (!isJunction(on) && on.type != (uint8_t)NodeType::START) continue;
    for (int r = 0; r < 4; ++r) {
      uint8_t need = rotMask(ex0, r);
      if ((on.exits & need) != need) continue;
      Alignment c;
      c.rot = r;
      c.tx = on.x;  // local node 0 is at the local origin
      c.ty = on.y;
      c.anchor_old = o;
      c.recency = on.odo;
      int matched = 0;
      if (!consistent(old_map, local, c, radius, matched)) continue;
      c.matched = matched;
      ++count;
      if (!have || matched > best.matched || (matched == best.matched && c.recency > best.recency)) {
        int prev = have ? best.matched : 0;
        int second = have ? (prev > best.runner_up ? prev : best.runner_up) : 0;
        best = c;
        best.runner_up = second;
        have = true;
      } else if (matched > best.runner_up) {
        best.runner_up = matched;
      }
    }
  }
  return count;
}

bool Relocalizer::align(const MazeGraph& old_map, const MazeGraph& local, float radius, Alignment& out) {
  int anchor = -1, n_local_j = 0;
  for (int i = 1; i < local.nodeCount(); ++i)
    if (isJunction(local.node(i))) {
      if (anchor < 0) anchor = i;
      ++n_local_j;
    }
  if (anchor < 0 || n_local_j < 3) return false;
  int found = 0;
  Alignment best;
  for (int r = 0; r < 4; ++r) {
    float ax, ay;
    rot(r, local.node(anchor).x, local.node(anchor).y, ax, ay);
    uint8_t aex = rotMask(local.node(anchor).exits, r);
    for (int o = 0; o < old_map.nodeCount(); ++o) {
      const Node& on = old_map.node(o);
      if (!isJunction(on) || on.exits != aex) continue;
      Alignment c;
      c.rot = r;
      c.tx = on.x - ax;
      c.ty = on.y - ay;
      int matched = 0;
      // Three re-observed junctions: two collinear T-junctions are too
      // common in a grid maze to place the robot on their own.
      if (consistent(old_map, local, c, radius, matched) && matched >= 3) {
        ++found;
        c.matched = matched;
        best = c;
      }
    }
  }
  if (found == 1) {
    out = best;
    return true;
  }
  if (found > 1) Log::printf("RELOC %d free alignments - need more junctions", found);
  return false;
}

bool Relocalizer::merge(MazeGraph& g, const MazeGraph& local, const Alignment& a, int* map) {
  map[0] = a.anchor_old;
  if (a.anchor_old >= 0) g.node(a.anchor_old).exits |= rotMask(local.node(0).exits, a.rot);
  for (int i = 1; i < local.nodeCount(); ++i) {
    const Node& ln = local.node(i);
    float qx, qy;
    transform(a, ln.x, ln.y, qx, qy);
    uint8_t ex = rotMask(ln.exits, a.rot);
    int hit = -1;
    if (ln.type != (uint8_t)NodeType::GOAL) {
      float hd = 150.0f;
      for (int k = 0; k < g.nodeCount(); ++k) {
        const Node& kn = g.node(k);
        if (kn.type == (uint8_t)NodeType::GOAL) continue;
        float d = sqrtf((kn.x - qx) * (kn.x - qx) + (kn.y - qy) * (kn.y - qy));
        if (d < hd && (kn.exits & ex) == ex) { hd = d; hit = k; }
      }
    } else if (g.goalNode() >= 0) {
      hit = g.goalNode();
    }
    if (hit < 0) {
      hit = g.addNode(qx, qy, ex, (NodeType)ln.type);
      if (hit < 0) return false;
      g.node(hit).odo = ln.odo;
      if (ln.type == (uint8_t)NodeType::GOAL) g.setGoal(hit);
    }
    map[i] = hit;
  }
  for (int e = 0; e < local.edgeCount(); ++e) {
    const Edge& le = local.edge(e);
    if (map[le.a] < 0 || map[le.b] < 0) continue;  // partial edge from an unanchored restart point
    int r = g.addEdge(map[le.a], dirAdd(le.da, a.rot), map[le.b], dirAdd(le.db, a.rot), le.length, le.seg_turn,
                      le.seg_len, le.nseg);
    if (r < 0) return false;
  }
  return true;
}

}  // namespace mesh
