// junction_detector.h - turns the stream of LineFeatures into discrete
// track events while line following.
//
// Geometry: the sensor line is sensor_forward_mm (L_s) AHEAD of the axle.
// When the array crosses a perpendicular 30 mm line between odometry
// distances s0 and s1, the junction centre is under the axle at
//     s_center = (s0 + s1) / 2 + L_s
// so the robot has L_s - (s1 - s0)/2 (~50 mm) to classify and brake before
// a pivot turn centred on the junction.
//
// Events
//   JUNCTION  exits {left, straight, right}. One side + no straight = corner.
//   DEAD_END  the line ended while the array was centred on it.
//   GOAL      the array stayed all-white for goal_confirm_mm (end box).
//   LINE_LOST the line vanished while the robot was off-centre (tracking error).
#pragma once
#include "../goal_detection/goal_detector.h"
#include "../sensors/line_sensor.h"
#include "config.h"

namespace mesh {

enum class EventType : uint8_t { NONE = 0, JUNCTION, DEAD_END, GOAL, LINE_LOST };

struct TrackEvent {
  EventType type = EventType::NONE;
  bool left = false, straight = false, right = false;
  float s_detect = 0;   // odometry s when the pattern was first seen
  float s_center = 0;   // odometry s at which the AXLE is over the feature
  float crossbar_mm = 0;
  float entry_pos_mm = 0;  // line position just before the crossing (tracking error)
};

class JunctionDetector {
 public:
  void init(const RobotConfig* cfg);
  // Re-arm after a turn or a start: patterns are ignored until a clean,
  // centred line has been seen for a few mm.
  void rearm(float s);
  void ignoreUntil(float s) { ignore_until_ = s; }
  bool update(const LineFeatures& f, float s, TrackEvent& ev);
  bool crossing() const { return state_ == State::CROSSING || state_ == State::LOOKAHEAD; }
  bool armed() const { return state_ != State::ARMING; }

 private:
  enum class State : uint8_t { ARMING, TRACK, CROSSING, LOOKAHEAD };
  const RobotConfig* cfg_ = nullptr;
  GoalDetector goal_;
  State state_ = State::ARMING;
  float clean_start_ = -1;
  float ignore_until_ = -1e9f;
  int branch_count_ = 0;
  float s0_ = 0, s1_ = 0, last_branch_s_ = 0;
  bool l_ = false, r_ = false;
  float look_end_ = 0;
  int votes_on_ = 0, votes_total_ = 0;
  float lost_start_ = -1;
  float last_pos_ = 0;
  float pos_ema_ = 0;
  float entry_pos_ = 0;
};

}  // namespace mesh
