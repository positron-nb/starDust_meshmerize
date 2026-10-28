// storage.h - non-volatile record of calibration, map and route.
//
// RAM does NOT survive a power cycle, so everything the actual run needs is
// serialised into one CRC-protected blob. On the ESP32 the blob lives in the
// NVS flash partition (Preferences API, written only while the robot is
// standing still: flash writes briefly stall both CPU cores). In the
// simulator it lives in a byte array that survives "power cycles".
//
// Layout: Header | SensorCalibration | Node[n] | Edge[m] | RouteStep[k]
// A wrong magic/version/size/CRC means "no data" and the robot falls back to
// a safe behaviour (fresh dry run, or exploration during the actual run).
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "../maze/graph.h"
#include "../planner/planner.h"
#include "../sensors/line_sensor.h"
#include "config.h"
#include "robot_types.h"

namespace mesh {

class IStorage {
 public:
  virtual ~IStorage() {}
  virtual size_t capacity() const = 0;
  virtual bool read(uint8_t* buf, size_t len) = 0;
  virtual bool write(const uint8_t* buf, size_t len) = 0;  // may take ~1-2 s on flash
};

struct PersistHeader {
  uint32_t magic;
  uint32_t format;
  uint32_t firmware;
  uint32_t payload_len;
  uint32_t crc;
  uint8_t stage;
  uint8_t n_nodes;
  uint8_t n_edges;
  uint8_t route_n;
  int16_t goal;
  uint8_t start_heading;
  uint8_t restarts_used;
  float route_length;
};

constexpr uint32_t kPersistMagic = 0x3148534Du;  // "MSH1"
constexpr uint32_t kPersistFormat = 4;
constexpr size_t kPersistMaxBytes = sizeof(PersistHeader) + sizeof(SensorCalibration) +
                                    sizeof(Node) * kMaxNodes + sizeof(Edge) * kMaxEdges +
                                    sizeof(RouteStep) * kMaxRouteSteps;

struct PersistState {
  Stage stage = Stage::FRESH;
  SensorCalibration cal{};
  MazeGraph* graph = nullptr;   // must point at a valid graph
  Route* route = nullptr;       // must point at a valid route
  uint8_t restarts_used = 0;
};

// Returns bytes written to buf (0 on error).
size_t persistSerialize(const PersistState& st, uint8_t* buf, size_t cap);
// Returns true if buf held a valid record (st filled), false otherwise.
bool persistDeserialize(PersistState& st, const uint8_t* buf, size_t len);

}  // namespace mesh
