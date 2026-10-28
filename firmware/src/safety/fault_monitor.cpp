#include "fault_monitor.h"

#include "../util/util.h"

namespace mesh {

FaultCode FaultMonitor::update(float commanded, float measured_abs, float vbat, float dt_s) {
  if (commanded > 60.0f && measured_abs < 8.0f) {
    stall_ms_ += dt_s * 1000.0f;
  } else {
    stall_ms_ = 0;
  }
  if (stall_ms_ >= cfg_->stall_ms) {
    stall_ms_ = 0;
    return FaultCode::STALL;
  }
  // Under heavy acceleration a 2S pack sags briefly; only a sustained low
  // voltage is reported (the run continues - stopping would cost more).
  if (vbat > 3.0f && vbat < cfg_->low_batt_v - 0.3f) {
    low_ms_ += dt_s * 1000.0f;
    if (low_ms_ > 2000.0f) {
      low_ms_ = -60000.0f;  // log at most once a minute
      Log::printf("WARN battery low %.2f V", vbat);
    }
  } else if (low_ms_ > 0) {
    low_ms_ = 0;
  }
  return FaultCode::NONE;
}

}  // namespace mesh
