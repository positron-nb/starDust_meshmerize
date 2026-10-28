// fault_monitor.h - stall and battery supervision.
//
// Stall: the controller is commanding motion but neither wheel has moved
// for stall_ms (robot jammed, wheel off the floor is NOT a stall - it would
// spin). The robot then stops its motors (protects the driver, the motor
// and the arena surface) and waits for a restart.
#pragma once
#include "config.h"
#include "robot_types.h"

namespace mesh {

class FaultMonitor {
 public:
  void init(const RobotConfig* cfg) { cfg_ = cfg; reset(); }
  void reset() { stall_ms_ = 0; low_ms_ = 0; }
  FaultCode update(float commanded, float measured_abs, float vbat, float dt_s);
  bool batteryOkToStart(float vbat) const { return vbat < 3.0f || vbat >= cfg_->low_batt_v; }

 private:
  const RobotConfig* cfg_ = nullptr;
  float stall_ms_ = 0;
  float low_ms_ = 0;
};

}  // namespace mesh
