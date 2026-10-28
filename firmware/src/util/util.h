// util.h - small helpers shared by all core modules (no heap, no STL).
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace mesh {

constexpr float kPi = 3.14159265358979f;
constexpr float kDeg = kPi / 180.0f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float absf(float v) { return v < 0 ? -v : v; }
inline float signf(float v) { return v < 0 ? -1.0f : 1.0f; }
inline float minf(float a, float b) { return a < b ? a : b; }
inline float maxf(float a, float b) { return a > b ? a : b; }

uint32_t crc32(const uint8_t* data, size_t len, uint32_t crc = 0);

// Event log: a fixed ring of short text lines. On the robot it is optionally
// mirrored to the USB serial port (debug builds only); in the simulator the
// lines are pulled through mesh_pop_log() and written to the run log.
class Log {
 public:
  static void printf(const char* fmt, ...);
  static bool pop(char* out, int cap);  // oldest line, false if empty
  static void clear();
  static void setTimeMs(uint32_t ms) { time_ms_ = ms; }
  typedef void (*Sink)(const char* line);
  static void setSink(Sink s) { sink_ = s; }

 private:
  static constexpr int kLines = 64;
  static constexpr int kLen = 120;
  static char buf_[kLines][kLen];
  static int head_, count_;
  static uint32_t time_ms_;
  static Sink sink_;
};

}  // namespace mesh
