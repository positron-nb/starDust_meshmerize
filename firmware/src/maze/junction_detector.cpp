#include "junction_detector.h"

#include "../util/util.h"

namespace mesh {

void JunctionDetector::init(const RobotConfig* cfg) {
  cfg_ = cfg;
  goal_.init(cfg);
  rearm(0);
}

void JunctionDetector::rearm(float s) {
  state_ = State::ARMING;
  clean_start_ = -1;
  branch_count_ = 0;
  lost_start_ = -1;
  goal_.reset();
  (void)s;
}

bool JunctionDetector::update(const LineFeatures& f, float s, TrackEvent& ev) {
  const bool branch = f.left_branch || f.right_branch;
  // The end box is tested on every sample, in every state: the robot may
  // enter it straight from a turn, a crossing or plain tracking.
  if (goal_.update(f.goal_white, s)) {
    ev = TrackEvent();
    ev.type = EventType::GOAL;
    ev.s_detect = goal_.whiteStart();
    ev.s_center = goal_.whiteStart() + cfg_->sensor_forward_mm;
    state_ = State::TRACK;
    goal_.reset();
    return true;
  }
  switch (state_) {
    case State::ARMING: {
      // Wait for a clean centred line (no branch pattern) for 6 mm.
      if (f.line_seen && f.center_on && !branch) {
        if (clean_start_ < 0) clean_start_ = s;
        if (s - clean_start_ >= 6.0f) {
          state_ = State::TRACK;
          lost_start_ = -1;
        }
      } else {
        clean_start_ = -1;
      }
      return false;
    }

    case State::TRACK: {
      if (f.line_seen) {
        lost_start_ = -1;
        last_pos_ = f.pos_mm;
        if (!branch) pos_ema_ += 0.2f * (f.pos_mm - pos_ema_);
      }
      if (branch && s >= ignore_until_) {
        if (++branch_count_ == 1) {
          s0_ = s;
          entry_pos_ = pos_ema_;
        }
        if (branch_count_ >= 2) {
          state_ = State::CROSSING;
          l_ = f.left_branch;
          r_ = f.right_branch;
          last_branch_s_ = s;
        }
        return false;
      }
      branch_count_ = 0;
      if (!f.line_seen) {
        if (lost_start_ < 0) lost_start_ = s;
        float gap = s - lost_start_;
        if (absf(last_pos_) > 0.5f * cfg_->sensor_pitch_mm * (cfg_->num_sensors - 2)) {
          // Line slid off the side of the array: tracking failure, not a dead end.
          if (gap >= cfg_->deadend_confirm_mm) {
            ev = TrackEvent();
            ev.type = EventType::LINE_LOST;
            ev.s_detect = lost_start_;
            ev.s_center = s;
            ev.left = last_pos_ < 0;
            ev.right = last_pos_ > 0;
            state_ = State::ARMING;
            return true;
          }
        } else if (gap >= cfg_->deadend_confirm_mm) {
          ev = TrackEvent();
          ev.type = EventType::DEAD_END;
          ev.s_detect = lost_start_;
          ev.s_center = lost_start_ + cfg_->sensor_forward_mm;  // where the line ends (axle frame)
          state_ = State::ARMING;
          return true;
        }
      }
      return false;
    }

    case State::CROSSING: {
      if (branch) {
        l_ = l_ || f.left_branch;
        r_ = r_ || f.right_branch;
        last_branch_s_ = s;
      } else if (s - last_branch_s_ >= cfg_->crossbar_clear_mm) {
        s1_ = last_branch_s_;
        state_ = State::LOOKAHEAD;
        look_end_ = s + cfg_->lookahead_mm;
        votes_on_ = votes_total_ = 0;
      }
      return false;
    }

    case State::LOOKAHEAD: {
      if (branch) {
        // Pattern came back (wide junction or box edge): keep crossing.
        state_ = State::CROSSING;
        l_ = l_ || f.left_branch;
        r_ = r_ || f.right_branch;
        last_branch_s_ = s;
        return false;
      }
      ++votes_total_;
      if (f.line_seen && f.center_on) ++votes_on_;
      if (s >= look_end_) {
        ev = TrackEvent();
        ev.type = EventType::JUNCTION;
        ev.left = l_;
        ev.right = r_;
        ev.straight = votes_on_ * 2 > votes_total_;
        ev.s_detect = s0_;
        ev.s_center = 0.5f * (s0_ + s1_) + cfg_->sensor_forward_mm;
        ev.crossbar_mm = s1_ - s0_;
        ev.entry_pos_mm = entry_pos_;
        state_ = State::TRACK;
        branch_count_ = 0;
        lost_start_ = ev.straight ? -1 : s;
        ignore_until_ = s1_ + cfg_->min_event_spacing_mm;
        last_pos_ = 0;
        return true;
      }
      return false;
    }
  }
  return false;
}

}  // namespace mesh
