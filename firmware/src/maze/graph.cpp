#include "graph.h"

#include <math.h>
#include <string.h>

#include "../util/util.h"

namespace mesh {

void MazeGraph::clear() {
  n_nodes_ = n_edges_ = 0;
  goal_ = -1;
}

int MazeGraph::addNode(float x, float y, uint8_t exits, NodeType type) {
  if (n_nodes_ >= kMaxNodes) return -1;
  Node& n = nodes_[n_nodes_];
  n.x = x;
  n.y = y;
  n.exits = exits;
  n.type = (uint8_t)type;
  n.visits = 1;
  n.flags = 0;
  n.odo = 0;
  for (int d = 0; d < 4; ++d) n.edge[d] = -1;
  return n_nodes_++;
}

int MazeGraph::matchNode(float x, float y, uint8_t exits, uint8_t arrival_exit, float base_radius, float drift_frac,
                         float max_radius, float odo_now) const {
  int best = -1, second = -1;
  float best_score = 1e9f, best_d = 1e9f, second_d = 1e9f;
  for (int i = 0; i < n_nodes_; ++i) {
    const Node& n = nodes_[i];
    if (n.type == (uint8_t)NodeType::GOAL) continue;
    float dx = n.x - x, dy = n.y - y;
    float d = sqrtf(dx * dx + dy * dy);
    float since = odo_now - n.odo;
    float radius = base_radius + drift_frac * (since > 0 ? since : 0);
    if (radius > max_radius) radius = max_radius;
    if (d > radius) continue;
    if (!(n.exits & dirBit(arrival_exit))) continue;  // must have the line we came in on
    float score = d;
    if (n.exits != exits) {
      // A missed/false side branch is possible, but only accept a mismatching
      // exit set when the position agrees well.
      if (d > 0.5f * radius) continue;
      score += 0.5f * radius;
    }
    if (score < best_score) {
      second = best;
      second_d = best_d;
      best_score = score;
      best = i;
      best_d = d;
    } else if (d < second_d) {
      second = i;
      second_d = d;
    }
  }
  // Ambiguity guard: two plausible candidates at similar distances -> do not
  // guess (a wrong loop closure corrupts the map; a duplicate node does not).
  if (best >= 0 && second >= 0 && second_d < 1.6f * best_d + 20.0f) return -1;
  return best;
}

int MazeGraph::addEdge(int a, uint8_t da, int b, uint8_t db, float length, const int8_t* turns,
                       const uint16_t* lens, uint8_t nseg) {
  if (a < 0 || b < 0) return -1;
  int existing = nodes_[a].edge[da];
  if (existing >= 0) {
    Edge& e = edges_[existing];
    // Re-traversal: refine the length estimate (running mean).
    float k = 1.0f / (float)(e.traversals + 1);
    e.length += k * (length - e.length);
    if (e.traversals < 255) ++e.traversals;
    return existing;
  }
  if (n_edges_ >= kMaxEdges) return -1;
  Edge& e = edges_[n_edges_];
  e.a = (uint8_t)a;
  e.da = da;
  e.b = (uint8_t)b;
  e.db = db;
  e.length = length;
  e.traversals = 1;
  if (nseg > kMaxEdgeSegs) nseg = kMaxEdgeSegs;
  e.nseg = nseg;
  for (int i = 0; i < kMaxEdgeSegs; ++i) {
    e.seg_turn[i] = i < nseg ? turns[i] : 0;
    e.seg_len[i] = i < nseg ? lens[i] : 0;
  }
  nodes_[a].edge[da] = (int16_t)n_edges_;
  nodes_[a].exits |= dirBit(da);
  nodes_[b].edge[db] = (int16_t)n_edges_;
  nodes_[b].exits |= dirBit(db);
  return n_edges_++;
}

int MazeGraph::otherEnd(int ei, int from, uint8_t from_dir, uint8_t* arrive_exit) const {
  const Edge& e = edges_[ei];
  if (e.a == from && e.da == from_dir) {
    if (arrive_exit) *arrive_exit = e.db;
    return e.b;
  }
  if (arrive_exit) *arrive_exit = e.da;
  return e.a;
}

uint8_t MazeGraph::arrivalHeading(int ei, int from) const {
  const Edge& e = edges_[ei];
  // Self-loop ambiguity is resolved by callers through otherEnd(); here a
  // traversal from a uses db, otherwise da.
  return e.a == from ? dirOpp(e.db) : dirOpp(e.da);
}

uint8_t MazeGraph::unexploredExits(int i) const {
  const Node& n = nodes_[i];
  uint8_t m = 0;
  for (int d = 0; d < 4; ++d)
    if ((n.exits & dirBit(d)) && n.edge[d] < 0) m |= dirBit(d);
  return m;
}

bool MazeGraph::hasFrontier() const {
  for (int i = 0; i < n_nodes_; ++i)
    if (nodes_[i].type != (uint8_t)NodeType::GOAL && unexploredExits(i)) return true;
  return false;
}

int MazeGraph::validate() const {
  int problems = 0;
  for (int i = 0; i < n_edges_; ++i) {
    const Edge& e = edges_[i];
    if (e.a >= n_nodes_ || e.b >= n_nodes_) {
      Log::printf("MAP edge %d bad endpoint", i);
      ++problems;
      continue;
    }
    if (nodes_[e.a].edge[e.da] != i || nodes_[e.b].edge[e.db] != i) {
      Log::printf("MAP edge %d not linked from its nodes", i);
      ++problems;
    }
    // Heading consistency: leaving a in da, each left corner is -1 in the
    // N/E/S/W index; we must arrive at b moving opposite to db.
    int h = e.da;
    for (int k = 0; k + 1 < e.nseg; ++k) h = dirAdd((uint8_t)h, -e.seg_turn[k]);
    if ((uint8_t)h != dirOpp(e.db)) {
      Log::printf("MAP edge %d heading mismatch", i);
      ++problems;
    }
    if (!(e.length > 0.0f)) {
      Log::printf("MAP edge %d zero length", i);
      ++problems;
    }
  }
  for (int i = 0; i < n_nodes_; ++i) {
    for (int d = 0; d < 4; ++d) {
      int ei = nodes_[i].edge[d];
      if (ei >= n_edges_) {
        Log::printf("MAP node %d dangling edge", i);
        ++problems;
      }
    }
  }
  return problems;
}

void MazeGraph::truncate(int n_nodes, int n_edges) {
  if (n_nodes < n_nodes_) n_nodes_ = n_nodes < 0 ? 0 : n_nodes;
  if (n_edges < n_edges_) n_edges_ = n_edges < 0 ? 0 : n_edges;
  for (int i = 0; i < n_nodes_; ++i)
    for (int d = 0; d < 4; ++d)
      if (nodes_[i].edge[d] >= n_edges_) nodes_[i].edge[d] = -1;
  if (goal_ >= n_nodes_) goal_ = -1;
}

void MazeGraph::mergeNodes(int keep, int drop) {
  if (keep == drop) return;
  Node& k = nodes_[keep];
  Node& d = nodes_[drop];
  for (int dir = 0; dir < 4; ++dir) {
    if (d.edge[dir] >= 0 && k.edge[dir] < 0) k.edge[dir] = d.edge[dir];
    k.exits |= d.exits;
  }
  for (int i = 0; i < n_edges_; ++i) {
    if (edges_[i].a == drop) edges_[i].a = (uint8_t)keep;
    if (edges_[i].b == drop) edges_[i].b = (uint8_t)keep;
  }
  // Leave the dropped node as an isolated, fully explored stub.
  d.exits = 0;
  for (int dir = 0; dir < 4; ++dir) d.edge[dir] = -1;
  if (goal_ == drop) goal_ = keep;
}

}  // namespace mesh
