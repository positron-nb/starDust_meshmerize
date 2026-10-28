#include "planner.h"

#include "../util/util.h"

namespace mesh {

namespace {
constexpr int kStates = kMaxNodes * 4;
float g_dist[kStates];
int16_t g_prev[kStates];   // previous state
int16_t g_prev_edge[kStates];
uint8_t g_prev_exit[kStates];
bool g_done[kStates];
}  // namespace

int Planner::turnCount(uint8_t heading, uint8_t exit) {
  uint8_t r = relTurn(heading, exit);
  return r == REL_STRAIGHT ? 0 : (r == REL_BACK ? 2 : 1);
}

void Planner::dijkstra(const MazeGraph& g, int start, uint8_t heading, float tw) {
  const int ns = g.nodeCount() * 4;
  for (int i = 0; i < ns; ++i) {
    g_dist[i] = 1e30f;
    g_prev[i] = -1;
    g_prev_edge[i] = -1;
    g_done[i] = false;
  }
  if (start < 0 || start >= g.nodeCount()) return;
  g_dist[start * 4 + heading] = 0;
  for (;;) {
    int u = -1;
    float best = 1e29f;
    for (int i = 0; i < ns; ++i)
      if (!g_done[i] && g_dist[i] < best) { best = g_dist[i]; u = i; }
    if (u < 0) break;
    g_done[u] = true;
    int node = u / 4;
    uint8_t h = (uint8_t)(u % 4);
    const Node& n = g.node(node);
    // The GOAL node is a sink: never route through it.
    if (n.type == (uint8_t)NodeType::GOAL && node != start) continue;
    for (uint8_t d = 0; d < 4; ++d) {
      int e = n.edge[d];
      if (e < 0) continue;
      int tc = turnCount(h, d);
      uint8_t arrive_exit;
      int m = g.otherEnd(e, node, d, &arrive_exit);
      uint8_t h2 = dirOpp(arrive_exit);
      int v = m * 4 + h2;
      float c = g_dist[u] + g.edge(e).length + tw * (float)(tc + g.edgeTurns(e));
      if (c < g_dist[v]) {
        g_dist[v] = c;
        g_prev[v] = (int16_t)u;
        g_prev_edge[v] = (int16_t)e;
        g_prev_exit[v] = d;
      }
    }
  }
}

static bool buildRoute(const MazeGraph& g, int target_state, uint8_t heading, Route& out) {
  // Walk back from the target state collecting steps.
  RouteStep tmp[kMaxRouteSteps];
  int n = 0;
  int s = target_state;
  out.length = 0;
  out.turns = 0;
  while (g_prev[s] >= 0) {
    if (n >= kMaxRouteSteps) return false;
    int p = g_prev[s];
    tmp[n].node = (uint8_t)(p / 4);
    tmp[n].exit = g_prev_exit[s];
    tmp[n].edge = g_prev_edge[s];
    out.length += g.edge(tmp[n].edge).length;
    out.turns += Planner::turnCount((uint8_t)(p % 4), tmp[n].exit) + g.edgeTurns(tmp[n].edge);
    ++n;
    s = p;
  }
  out.n = n;
  for (int i = 0; i < n; ++i) out.steps[i] = tmp[n - 1 - i];
  out.start_heading = heading;
  out.valid = true;
  return true;
}

bool Planner::shortest(const MazeGraph& g, int start, uint8_t heading, int goal, float tw, Route& out) {
  out.valid = false;
  out.n = 0;
  if (goal < 0 || goal >= g.nodeCount()) return false;
  dijkstra(g, start, heading, tw);
  int best = -1;
  float bd = 1e29f;
  for (int h = 0; h < 4; ++h)
    if (g_dist[goal * 4 + h] < bd) { bd = g_dist[goal * 4 + h]; best = goal * 4 + h; }
  if (best < 0) return false;
  if (best == start * 4 + heading) {  // already there
    out.n = 0;
    out.valid = true;
    out.start_heading = heading;
    out.length = 0;
    return true;
  }
  return buildRoute(g, best, heading, out);
}

int Planner::nearestFrontier(const MazeGraph& g, int start, uint8_t heading, float tw, Route& out) {
  out.valid = false;
  out.n = 0;
  dijkstra(g, start, heading, tw);
  int best_state = -1;
  float bd = 1e29f;
  for (int i = 0; i < g.nodeCount(); ++i) {
    if (g.node(i).type == (uint8_t)NodeType::GOAL) continue;
    uint8_t un = g.unexploredExits(i);
    if (!un) continue;
    for (int h = 0; h < 4; ++h) {
      float c = g_dist[i * 4 + h];
      if (c >= 1e29f) continue;
      // cheapest turn onto any unexplored exit from this arrival heading
      int tmin = 3;
      for (int d = 0; d < 4; ++d)
        if (un & dirBit(d)) { int t = turnCount((uint8_t)h, (uint8_t)d); if (t < tmin) tmin = t; }
      c += tw * tmin;
      if (c < bd) { bd = c; best_state = i * 4 + h; }
    }
  }
  if (best_state < 0) return -1;
  if (!buildRoute(g, best_state, heading, out)) return -1;
  return best_state / 4;
}

void Planner::distancesFrom(const MazeGraph& g, int start, uint8_t heading, float tw, float* out) {
  dijkstra(g, start, heading, tw);
  for (int i = 0; i < g.nodeCount(); ++i) {
    float d = 1e30f;
    for (int h = 0; h < 4; ++h) d = minf(d, g_dist[i * 4 + h]);
    out[i] = d;
  }
}

int Planner::bestFrontier(const MazeGraph& g, int robot, uint8_t heading, float tw, float start_weight, Route& out) {
  static float from_start[kMaxNodes];
  out.valid = false;
  out.n = 0;
  if (start_weight > 0) distancesFrom(g, 0, DIR_N, 0.0f, from_start);
  dijkstra(g, robot, heading, tw);
  int best_state = -1;
  float bd = 1e29f;
  for (int i = 0; i < g.nodeCount(); ++i) {
    if (g.node(i).type == (uint8_t)NodeType::GOAL) continue;
    uint8_t un = g.unexploredExits(i);
    if (!un) continue;
    float bias = start_weight > 0 && from_start[i] < 1e29f ? start_weight * from_start[i] : 0.0f;
    for (int h = 0; h < 4; ++h) {
      float c = g_dist[i * 4 + h];
      if (c >= 1e29f) continue;
      int tmin = 3;
      for (int d = 0; d < 4; ++d)
        if (un & dirBit(d)) { int t = turnCount((uint8_t)h, (uint8_t)d); if (t < tmin) tmin = t; }
      c += tw * tmin + bias;
      if (c < bd) { bd = c; best_state = i * 4 + h; }
    }
  }
  if (best_state < 0) return -1;
  if (best_state / 4 == robot) {
    out.valid = true;  // the best frontier is right here
    return robot;
  }
  if (!buildRoute(g, best_state, heading, out)) return -1;
  return best_state / 4;
}

float Planner::unexploredLowerBound(const MazeGraph& g, int start, uint8_t heading, int goal) {
  if (goal < 0) return 0;
  dijkstra(g, start, heading, 0.0f);
  const Node& G = g.node(goal);
  float lb = 1e30f;
  for (int i = 0; i < g.nodeCount(); ++i) {
    if (i == goal || !g.unexploredExits(i)) continue;
    float d = 1e30f;
    for (int h = 0; h < 4; ++h) d = minf(d, g_dist[i * 4 + h]);
    if (d >= 1e29f) continue;
    float m = absf(g.node(i).x - G.x) + absf(g.node(i).y - G.y);
    lb = minf(lb, d + m);
  }
  return lb;
}

}  // namespace mesh
