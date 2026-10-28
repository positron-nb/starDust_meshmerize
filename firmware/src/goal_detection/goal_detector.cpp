#include "goal_detector.h"

namespace mesh {

bool GoalDetector::update(bool all_white, float s) {
  if (!all_white) {
    white_start_ = -1;
    return false;
  }
  if (white_start_ < 0) white_start_ = s;
  return (s - white_start_) >= cfg_->goal_confirm_mm;
}

int GoalDetector::updateConfirm(bool mostly_white, float s) {
  if (!mostly_white) {
    if (loss_start_ < 0) loss_start_ = s;
    if (s - loss_start_ >= cfg_->goal_abort_loss_mm) return -1;
  } else {
    loss_start_ = -1;
  }
  if (s - confirm_start_ >= cfg_->goal_stop_extra_mm) return +1;
  return 0;
}

}  // namespace mesh
