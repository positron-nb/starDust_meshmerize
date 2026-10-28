// sim_api.cpp - C ABI used by the Python simulator (ctypes) and the host
// unit tests. Compiled ONLY into the PC shared library (MESH_SIM defined);
// the ESP32 firmware never contains this file's code.
//
// The simulator drives the real Robot object: it computes sensor/encoder
// readings from its physics model, calls mesh_step() once per 1 ms tick and
// applies the returned motor duties. Planning code therefore never sees the
// simulator's ground-truth maze.
#ifdef MESH_SIM

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "../app/robot.h"
#include "../util/util.h"

#if defined(_WIN32)
#define MESH_API extern "C" __declspec(dllexport)
#else
#define MESH_API extern "C" __attribute__((visibility("default")))
#endif

using namespace mesh;

namespace {
struct SimHandle {
  Robot robot;
  uint8_t storage[kPersistMaxBytes + 64];
  size_t storage_len = 0;
  int saves = 0;
};

struct ParamDesc {
  const char* name;
  size_t offset;
  bool is_int;
};

const ParamDesc kParams[] = {
#define MESH_CFG_DESC(type, name, def, doc) \
  {#name, offsetof(RobotConfig, name), sizeof(type) == 4 && (type)0.5f == 0},
    MESH_CONFIG_FIELDS(MESH_CFG_DESC)
#undef MESH_CFG_DESC
};

const ParamDesc* findParam(const char* key) {
  for (const ParamDesc& p : kParams)
    if (strcmp(p.name, key) == 0) return &p;
  return nullptr;
}

// Small append-only JSON writer.
struct Json {
  char* buf;
  int cap;
  int n = 0;
  void put(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
};
void Json::put(const char* fmt, ...) {
  if (n >= cap) return;
  va_list ap;
  va_start(ap, fmt);
  int w = vsnprintf(buf + n, cap - n, fmt, ap);
  va_end(ap);
  if (w > 0) n += w;
  if (n > cap) n = cap;
}
}  // namespace

MESH_API void* mesh_new(void) {
  SimHandle* h = new SimHandle();
  h->robot.init();
  return h;
}

MESH_API void mesh_free(void* p) { delete static_cast<SimHandle*>(p); }

MESH_API int mesh_param_count(void) { return (int)(sizeof(kParams) / sizeof(kParams[0])); }

MESH_API const char* mesh_param_name(int i) {
  return (i >= 0 && i < mesh_param_count()) ? kParams[i].name : nullptr;
}

MESH_API int mesh_set_param(void* p, const char* key, double v) {
  SimHandle* h = static_cast<SimHandle*>(p);
  const ParamDesc* d = findParam(key);
  if (!d) return -1;
  uint8_t* base = reinterpret_cast<uint8_t*>(&h->robot.cfg) + d->offset;
  if (d->is_int) *reinterpret_cast<int32_t*>(base) = (int32_t)v;
  else *reinterpret_cast<float*>(base) = (float)v;
  return 0;
}

MESH_API double mesh_get_param(void* p, const char* key) {
  SimHandle* h = static_cast<SimHandle*>(p);
  const ParamDesc* d = findParam(key);
  if (!d) return -1e300;
  uint8_t* base = reinterpret_cast<uint8_t*>(&h->robot.cfg) + d->offset;
  return d->is_int ? (double)*reinterpret_cast<int32_t*>(base) : (double)*reinterpret_cast<float*>(base);
}

// Simulated power-up: loads whatever is in the storage array.
MESH_API void mesh_boot(void* p) {
  SimHandle* h = static_cast<SimHandle*>(p);
  h->robot.loadPersisted(h->storage, h->storage_len);
}

MESH_API void mesh_step(void* p, const HalInputs* in, HalOutputs* out) {
  SimHandle* h = static_cast<SimHandle*>(p);
  h->robot.step(*in, *out);
  if (out->save_request) {
    h->storage_len = h->robot.serialize(h->storage, sizeof(h->storage));
    ++h->saves;
  }
}

MESH_API int mesh_state(void* p) { return (int)static_cast<SimHandle*>(p)->robot.state(); }
MESH_API int mesh_stage(void* p) { return (int)static_cast<SimHandle*>(p)->robot.stage(); }
MESH_API int mesh_fault(void* p) { return (int)static_cast<SimHandle*>(p)->robot.fault(); }
MESH_API int mesh_run_kind(void* p) { return (int)static_cast<SimHandle*>(p)->robot.runKind(); }
MESH_API int mesh_saves(void* p) { return static_cast<SimHandle*>(p)->saves; }

MESH_API int mesh_storage_get(void* p, uint8_t* buf, int cap) {
  SimHandle* h = static_cast<SimHandle*>(p);
  int n = (int)h->storage_len < cap ? (int)h->storage_len : cap;
  memcpy(buf, h->storage, n);
  return (int)h->storage_len;
}

MESH_API int mesh_storage_set(void* p, const uint8_t* buf, int len) {
  SimHandle* h = static_cast<SimHandle*>(p);
  if (len < 0 || (size_t)len > sizeof(h->storage)) return -1;
  memcpy(h->storage, buf, len);
  h->storage_len = (size_t)len;
  return 0;
}

MESH_API int mesh_pop_log(char* buf, int cap) { return Log::pop(buf, cap) ? (int)strlen(buf) : 0; }
MESH_API void mesh_clear_log(void) { Log::clear(); }

// JSON snapshot of the robot's belief (map, route, pose) for the visualiser
// and the test harness.
MESH_API int mesh_export_json(void* p, char* buf, int cap) {
  SimHandle* h = static_cast<SimHandle*>(p);
  const Robot& r = h->robot;
  const MazeGraph& g = r.graph();
  Json j{buf, cap};
  j.put("{\"state\":%d,\"stage\":%d,\"fault\":%d,\"run\":%d,\"heading\":%d,\"x\":%.1f,\"y\":%.1f,", (int)r.state(),
        (int)r.stage(), (int)r.fault(), (int)r.runKind(), r.heading(), r.poseX(), r.poseY());
  j.put("\"reloc\":%d,\"restarts\":%d,\"led\":%d,\"goal\":%d,\"route_idx\":%d,\"lower_bound\":%.1f,",
        r.relocalizing() ? 1 : 0, r.restartsUsed(), r.ledRed() ? 1 : 0, g.goalNode(), r.routeIndex(),
        r.lowerBound());
  j.put("\"nodes\":[");
  for (int i = 0; i < g.nodeCount(); ++i) {
    const Node& n = g.node(i);
    j.put("%s[%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d]", i ? "," : "", n.x, n.y, n.exits, n.type, n.visits, n.edge[0],
          n.edge[1], n.edge[2], n.edge[3]);
  }
  j.put("],\"edges\":[");
  for (int i = 0; i < g.edgeCount(); ++i) {
    const Edge& e = g.edge(i);
    j.put("%s{\"a\":%d,\"da\":%d,\"b\":%d,\"db\":%d,\"len\":%.1f,\"segs\":[", i ? "," : "", e.a, e.da, e.b, e.db,
          e.length);
    for (int k = 0; k < e.nseg; ++k) j.put("%s[%d,%d]", k ? "," : "", e.seg_len[k], e.seg_turn[k]);
    j.put("]}");
  }
  const Route& rt = r.route();
  j.put("],\"route\":{\"valid\":%d,\"length\":%.1f,\"turns\":%d,\"steps\":[", rt.valid ? 1 : 0, rt.length, rt.turns);
  for (int i = 0; i < rt.n; ++i)
    j.put("%s[%d,%d,%d]", i ? "," : "", rt.steps[i].node, rt.steps[i].exit, rt.steps[i].edge);
  j.put("]}}");
  if (j.n >= cap) return -1;
  return j.n;
}

// ------------------------------------------------- pure algorithm test API --
MESH_API void* mesh_graph_new(void) {
  MazeGraph* g = new MazeGraph();
  g->clear();
  return g;
}
MESH_API void mesh_graph_free(void* g) { delete static_cast<MazeGraph*>(g); }
MESH_API int mesh_graph_add_node(void* g, float x, float y, int exits, int type) {
  return static_cast<MazeGraph*>(g)->addNode(x, y, (uint8_t)exits, (NodeType)type);
}
MESH_API void mesh_graph_set_goal(void* g, int n) { static_cast<MazeGraph*>(g)->setGoal(n); }
MESH_API int mesh_graph_add_edge(void* g, int a, int da, int b, int db, float len, int corners) {
  int8_t turns[kMaxEdgeSegs] = {0};
  uint16_t lens[kMaxEdgeSegs] = {0};
  int nseg = corners + 1;
  if (nseg > kMaxEdgeSegs) nseg = kMaxEdgeSegs;
  // Synthetic polyline: equal segments; the corner turns alternate and the
  // last one is chosen so that we leave in da and arrive moving opposite db.
  // (Callers must request a corner count that makes this possible.)
  for (int i = 0; i < nseg; ++i) lens[i] = (uint16_t)(len / nseg);
  int net_cw = 0;
  for (int i = 0; i + 1 < nseg; ++i) {
    int t;
    if (i + 2 < nseg) {
      t = (i % 2 == 0) ? +1 : -1;
    } else {
      int need = ((int)dirOpp((uint8_t)db) - da - net_cw + 8) & 3;  // clockwise quarter turns still needed
      t = need == 3 ? +1 : -1;
    }
    turns[i] = (int8_t)t;
    net_cw -= t;
  }
  return static_cast<MazeGraph*>(g)->addEdge(a, (uint8_t)da, b, (uint8_t)db, len, turns, lens, (uint8_t)nseg);
}
MESH_API int mesh_graph_match(void* g, float x, float y, int exits, int arrive_exit, float radius) {
  return static_cast<MazeGraph*>(g)->matchNode(x, y, (uint8_t)exits, (uint8_t)arrive_exit, radius);
}
MESH_API int mesh_graph_validate(void* g) { return static_cast<MazeGraph*>(g)->validate(); }

// Plans start->goal. Writes up to `cap` (node, exit, edge) triples into
// out[]. Returns step count, or -1 if no route. length/turns are optional.
MESH_API int mesh_plan(void* g, int start, int heading, int goal, float turn_weight, int* out, int cap,
                       float* length, int* turns) {
  Route r;
  if (!Planner::shortest(*static_cast<MazeGraph*>(g), start, (uint8_t)heading, goal, turn_weight, r)) return -1;
  for (int i = 0; i < r.n && i < cap; ++i) {
    out[3 * i] = r.steps[i].node;
    out[3 * i + 1] = r.steps[i].exit;
    out[3 * i + 2] = r.steps[i].edge;
  }
  if (length) *length = r.length;
  if (turns) *turns = r.turns;
  return r.n;
}

MESH_API float mesh_lower_bound(void* g, int start, int heading, int goal) {
  return Planner::unexploredLowerBound(*static_cast<MazeGraph*>(g), start, (uint8_t)heading, goal);
}

MESH_API int mesh_nearest_frontier(void* g, int start, int heading) {
  Route r;
  return Planner::nearestFrontier(*static_cast<MazeGraph*>(g), start, (uint8_t)heading, 140.0f, r);
}

// Explorer decision with a given preference (policy 0, pref 0..5).
MESH_API int mesh_explore_decide(void* g, int node, int heading, int policy, int pref) {
  static RobotConfig cfg;
  cfg.explore_policy = policy;
  cfg.explore_pref = pref;
  Explorer ex;
  ex.init(&cfg);
  return ex.decide(*static_cast<MazeGraph*>(g), node, (uint8_t)heading);
}

// Serialisation round trip on a graph (returns bytes, 0 on failure).
MESH_API int mesh_persist_roundtrip(void* g, int corrupt_offset) {
  static uint8_t buf[kPersistMaxBytes];
  Route r;
  MazeGraph* mg = static_cast<MazeGraph*>(g);
  if (mg->goalNode() >= 0) Planner::shortest(*mg, 0, DIR_N, mg->goalNode(), 30.0f, r);
  PersistState st;
  st.stage = Stage::DRY_DONE;
  st.graph = mg;
  st.route = &r;
  size_t n = persistSerialize(st, buf, sizeof(buf));
  if (!n) return 0;
  if (corrupt_offset >= 0 && (size_t)corrupt_offset < n) buf[corrupt_offset] ^= 0x5A;
  MazeGraph g2;
  Route r2;
  PersistState st2;
  st2.graph = &g2;
  st2.route = &r2;
  if (!persistDeserialize(st2, buf, n)) return 0;
  if (g2.nodeCount() != mg->nodeCount() || g2.edgeCount() != mg->edgeCount() || r2.n != r.n) return 0;
  return (int)n;
}

#endif  // MESH_SIM
