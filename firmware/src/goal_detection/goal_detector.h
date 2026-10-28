// goal_detector.h - end-zone (400 x 400 mm white box) recognition.
//
// A junction also turns the whole array white, but only while the 30 mm
// cross line passes under it (about 30-40 mm of travel). The end box keeps
// the array white for ~400 mm. The detector therefore requires:
//   1. (nearly) all channels white,              - spatial pattern
//   2. continuously for >= goal_confirm_mm,       - persistence by DISTANCE
//                                                   (speed independent)
//   3. then a confirmation drive of goal_stop_extra_mm during which white
//      must not disappear for more than goal_abort_loss_mm.
// Only after step 2 is the red LED switched on; step 3 places the robot
// body inside the box and stops it. If step 3 fails the event is withdrawn
// (false goal) and the robot reverses to the junction it overshot.
#pragma once
#include "config.h"

namespace mesh {

class GoalDetector {
 public:
  void init(const RobotConfig* cfg) { cfg_ = cfg; reset(); }
  void reset() { white_start_ = -1; }
  // Phase 1+2: feed every sample. Returns true once the white run is long enough.
  bool update(bool all_white, float s);
  float whiteStart() const { return white_start_; }
  float whiteLength(float s) const { return white_start_ < 0 ? 0 : s - white_start_; }

  // Phase 3: confirmation drive.
  void beginConfirm(float s) { confirm_start_ = s; loss_start_ = -1; }
  // Returns +1 confirmed, -1 aborted (false goal), 0 still driving.
  int updateConfirm(bool mostly_white, float s);

 private:
  const RobotConfig* cfg_ = nullptr;
  float white_start_ = -1;
  float confirm_start_ = 0, loss_start_ = -1;
};

}  // namespace mesh
