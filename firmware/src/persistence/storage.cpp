#include "storage.h"

#include <string.h>

#include "../util/util.h"

namespace mesh {

size_t persistSerialize(const PersistState& st, uint8_t* buf, size_t cap) {
  const MazeGraph& g = *st.graph;
  const Route& r = *st.route;
  size_t payload = sizeof(SensorCalibration) + sizeof(Node) * g.nodeCount() + sizeof(Edge) * g.edgeCount() +
                   sizeof(RouteStep) * (r.valid ? r.n : 0);
  size_t total = sizeof(PersistHeader) + payload;
  if (total > cap) return 0;
  PersistHeader h;
  memset(&h, 0, sizeof(h));
  h.magic = kPersistMagic;
  h.format = kPersistFormat;
  h.firmware = kFirmwareVersion;
  h.payload_len = (uint32_t)payload;
  h.stage = (uint8_t)st.stage;
  h.n_nodes = (uint8_t)g.nodeCount();
  h.n_edges = (uint8_t)g.edgeCount();
  h.route_n = (uint8_t)(r.valid ? r.n : 0);
  h.goal = (int16_t)g.goalNode();
  h.start_heading = r.start_heading;
  h.restarts_used = st.restarts_used;
  h.route_length = r.valid ? r.length : 0;
  uint8_t* p = buf + sizeof(PersistHeader);
  memcpy(p, &st.cal, sizeof(SensorCalibration));
  p += sizeof(SensorCalibration);
  for (int i = 0; i < g.nodeCount(); ++i, p += sizeof(Node)) memcpy(p, &g.node(i), sizeof(Node));
  for (int i = 0; i < g.edgeCount(); ++i, p += sizeof(Edge)) memcpy(p, &g.edge(i), sizeof(Edge));
  for (int i = 0; i < h.route_n; ++i, p += sizeof(RouteStep)) memcpy(p, &r.steps[i], sizeof(RouteStep));
  h.crc = crc32(buf + sizeof(PersistHeader), payload);
  memcpy(buf, &h, sizeof(h));
  return total;
}

bool persistDeserialize(PersistState& st, const uint8_t* buf, size_t len) {
  if (len < sizeof(PersistHeader)) return false;
  PersistHeader h;
  memcpy(&h, buf, sizeof(h));
  if (h.magic != kPersistMagic || h.format != kPersistFormat) return false;
  if (h.firmware != kFirmwareVersion) {
    Log::printf("STORE record from other firmware %08lx ignored", (unsigned long)h.firmware);
    return false;
  }
  if (h.n_nodes > kMaxNodes || h.n_edges > kMaxEdges || h.route_n > kMaxRouteSteps) return false;
  size_t payload = sizeof(SensorCalibration) + sizeof(Node) * h.n_nodes + sizeof(Edge) * h.n_edges +
                   sizeof(RouteStep) * h.route_n;
  if (h.payload_len != payload || sizeof(PersistHeader) + payload > len) return false;
  if (crc32(buf + sizeof(PersistHeader), payload) != h.crc) {
    Log::printf("STORE CRC mismatch - record rejected");
    return false;
  }
  if (h.stage > (uint8_t)Stage::ACTUAL_DONE) return false;
  const uint8_t* p = buf + sizeof(PersistHeader);
  memcpy(&st.cal, p, sizeof(SensorCalibration));
  p += sizeof(SensorCalibration);
  MazeGraph& g = *st.graph;
  g.clear();
  for (int i = 0; i < h.n_nodes; ++i, p += sizeof(Node)) {
    Node n;
    memcpy(&n, p, sizeof(Node));
    int id = g.addNode(n.x, n.y, n.exits, (NodeType)n.type);
    g.node(id) = n;
    for (int d = 0; d < 4; ++d) g.node(id).edge[d] = -1;  // relinked by addEdge below
  }
  // Edges are re-added in the original order, so indices are preserved.
  for (int i = 0; i < h.n_edges; ++i, p += sizeof(Edge)) {
    Edge e;
    memcpy(&e, p, sizeof(Edge));
    if (e.a >= h.n_nodes || e.b >= h.n_nodes) return false;
    int id = g.addEdge(e.a, e.da, e.b, e.db, e.length, e.seg_turn, e.seg_len, e.nseg);
    (void)id;
  }
  g.setGoal(h.goal);
  Route& r = *st.route;
  r.n = h.route_n;
  for (int i = 0; i < h.route_n; ++i, p += sizeof(RouteStep)) {
    memcpy(&r.steps[i], p, sizeof(RouteStep));
    if (r.steps[i].node >= h.n_nodes || r.steps[i].edge < 0 || r.steps[i].edge >= h.n_edges) return false;
  }
  if (h.goal >= (int)h.n_nodes) return false;
  r.start_heading = h.start_heading;
  r.length = h.route_length;
  r.valid = h.route_n > 0;
  st.stage = (Stage)h.stage;
  st.restarts_used = h.restarts_used;
  return true;
}

}  // namespace mesh
