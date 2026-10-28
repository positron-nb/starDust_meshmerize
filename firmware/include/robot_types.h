// robot_types.h - shared plain-data types for the Meshmerize robot core.
//
// Everything in here is POD so it can be shared verbatim between the ESP32
// firmware, the host-side unit tests and the Python simulator (via ctypes).
// Do not reorder fields of HalInputs/HalOutputs without updating
// simulator/robot/core_bridge.py.
#pragma once
#include <stdint.h>

namespace mesh {

constexpr int kMaxSensors = 16;

// Absolute (map-frame) headings. N is the robot heading at the START point,
// E is a right turn from there. The map frame is x = east, y = north, in mm.
enum Dir : uint8_t { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3, DIR_NONE = 0xFF };

// Relative turn = (absolute exit - current heading) & 3
enum RelTurn : uint8_t { REL_STRAIGHT = 0, REL_RIGHT = 1, REL_BACK = 2, REL_LEFT = 3 };

inline uint8_t dirAdd(uint8_t d, int k) { return (uint8_t)((d + k + 8) & 3); }
inline uint8_t dirOpp(uint8_t d) { return (uint8_t)((d + 2) & 3); }
inline uint8_t dirBit(uint8_t d) { return (uint8_t)(1u << (d & 3)); }
inline uint8_t relTurn(uint8_t heading, uint8_t absExit) { return (uint8_t)((absExit - heading + 4) & 3); }
inline float dirDx(uint8_t d) { return d == DIR_E ? 1.0f : (d == DIR_W ? -1.0f : 0.0f); }
inline float dirDy(uint8_t d) { return d == DIR_N ? 1.0f : (d == DIR_S ? -1.0f : 0.0f); }
inline char relTurnChar(uint8_t r) { return "SRBL"[r & 3]; }
inline char dirChar(uint8_t d) { return d < 4 ? "NESW"[d] : '?'; }

// ---------------------------------------------------------------- HAL I/O --
// One snapshot of everything the robot can sense. Filled by the hardware
// layer (hal_esp32) on the robot or by the simulator on the PC.
struct HalInputs {
  uint32_t t_us;                 // monotonic microseconds (wraps after ~71 min)
  uint16_t raw[kMaxSensors];     // raw 12-bit readings: line array first, then the 2 wing sensors
                                 // (digital IR modules are read as 0 / 4095)
  int32_t enc_left;              // accumulated encoder counts (encoder_mode 0: signed quadrature,
  int32_t enc_right;             //   1: slot-disc pulses, always counting up; 2: unused)
  float vbat;                    // battery voltage, volts (0 if not measured)
  uint8_t button;                // 1 while the start button is held down
  uint8_t gyro_ok;               // 1 if gyro_z_dps is valid (MPU6050 present and read)
  uint8_t pad[2];
  float gyro_z_dps;              // yaw rate from the MPU6050, deg/s, CCW positive (after gyro_sign)
};

// Everything the robot core commands. Duty is signed, -1..+1, already
// corrected for motor polarity and limited to the motor voltage rating.
struct HalOutputs {
  float duty_left;
  float duty_right;
  uint8_t led_red;       // end-zone LED (rule: must glow at the end zone)
  uint8_t led_status;    // on-board status LED
  uint8_t emitters_on;   // reflectance emitter enable
  uint8_t motors_enable; // TB6612 STBY
  uint8_t save_request;  // 1 = core asks the HAL to persist storage now (robot stopped)
  uint8_t pad[3];
};

// ------------------------------------------------------- competition state --
enum class Stage : uint8_t {
  FRESH = 0,          // no dry-run data: next start = dry run
  DRY_PARTIAL = 1,    // dry run started but not completed (restart pending)
  DRY_DONE = 2,       // map + route stored: next start = actual run
  ACTUAL_DONE = 3,    // actual run completed
};

enum class RobotState : uint8_t {
  BOOT = 0,
  IDLE,               // waiting for button; LED pattern shows stage
  CALIBRATING,        // sweeping over the line collecting min/max
  ARMED,              // button released, short delay before moving
  DRY_RUN,            // exploring
  GOAL_CONFIRMATION,  // driving into the end box
  MAP_VALIDATION,
  ROUTE_PLANNING,
  WAIT_FOR_ACTUAL_RUN,
  ACTUAL_RUN,
  FINISHED,
  RECOVERY,           // automatic line-loss / fault recovery attempt
  FAULT,              // motors off, waiting for a restart (button)
  CLEAR_MEMORY,
};

enum class FaultCode : uint8_t {
  NONE = 0,
  LINE_LOST,
  STALL,
  LOW_BATTERY,
  ROUTE_MISMATCH,
  MAP_FULL,
  NO_FRONTIER,        // explored everything and never saw the end box
  TURN_FAILED,
  CALIBRATION_BAD,
  STORAGE,
  TIMEOUT,
};

enum class ExplorePolicy : uint8_t {
  DFS_NEAREST_FRONTIER = 0,  // Tremaux-style graph DFS with nearest-frontier backtracking (default)
  LEFT_HAND = 1,             // left-hand rule (map only used for loop escape)
  RIGHT_HAND = 2,            // right-hand rule
};

}  // namespace mesh
