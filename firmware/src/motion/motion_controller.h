// motion_controller.h - closed-loop motion primitives built on wheel encoders.
//
// Primitives (one active at a time):
//   FOLLOW   - PD line following with a trapezoidal speed profile and an
//              optional stop point (s_stop, v_end) so the robot brakes onto
//              a known junction position.  Steering can be frozen to an
//              encoder heading hold while crossing a junction.
//   STRAIGHT - blind, encoder-only drive of a signed distance (heading hold).
//   PIVOT    - in-place rotation by an angle, optionally finished by
//              "capturing" the new line with the reflectance array.
//   BRAKE    - ramp both wheels to zero.
// The output is a signed duty per wheel produced by the per-wheel PI speed
// controller with battery-voltage feed-forward.
#pragma once
#include "../encoders/odometry.h"
#include "../sensors/line_sensor.h"
#include "config.h"
#include "robot_types.h"

namespace mesh {

enum class MotionMode : uint8_t { IDLE, FOLLOW, STRAIGHT, PIVOT, BRAKE };

class WheelSpeedController {
 public:
  void init(const RobotConfig* cfg) { cfg_ = cfg; reset(); }
  void reset() { i_l_ = i_r_ = 0; }
  // Returns duty -1..1 for each wheel (motor polarity applied by caller).
  void update(float vref_l, float vref_r, float vmeas_l, float vmeas_r, float vbat, float dt,
              float& duty_l, float& duty_r);

 private:
  float one(float vref, float vmeas, float vbat, float dt, float& integ);
  const RobotConfig* cfg_ = nullptr;
  float i_l_ = 0, i_r_ = 0;
};

class MotionController {
 public:
  void init(const RobotConfig* cfg, const Odometry* odo);

  void idle();
  void brake(float decel_mm_s2);
  void follow(float v_cruise, float accel, float decel);
  void setCruise(float v_cruise) { v_cruise_ = v_cruise; }
  void setStopPoint(float s_stop, float v_end) { s_stop_ = s_stop; v_end_ = v_end; has_stop_ = true; }
  void clearStopPoint() { has_stop_ = false; }
  void holdHeading(bool on);
  void straight(float dist_mm, float v, float accel, float decel);
  // angle_rad > 0 = counter-clockwise (left). capture_after_rad < 0 disables capture.
  void pivot(float angle_rad, float omega_max, float alpha, float capture_after_rad, float overshoot_rad);

  void update(const LineFeatures& f, float dt, float vbat, HalOutputs& out);

  MotionMode mode() const { return mode_; }
  bool done() const { return done_; }
  bool failed() const { return failed_; }
  float vCmd() const { return v_cmd_; }
  float pivotRotated() const;  // rad, signed
  float commandedSpeed() const { return absf_(v_cmd_) + absf_(w_cmd_) * 50.0f; }
  float refLeft() const { return ref_l_; }   // wheel speeds commanded in the last update (mm/s)
  float refRight() const { return ref_r_; }
  float motorTrim() const { return trim_; }  // learned left/right gain balance (no-encoder mode)

 private:
  static float absf_(float v) { return v < 0 ? -v : v; }
  void wheelsFrom(float v, float w, float& vl, float& vr) const;

  const RobotConfig* cfg_ = nullptr;
  const Odometry* odo_ = nullptr;
  WheelSpeedController wheels_;
  MotionMode mode_ = MotionMode::IDLE;
  bool done_ = true, failed_ = false;

  // FOLLOW / STRAIGHT profile
  float v_cruise_ = 0, accel_ = 1000, decel_ = 1000, v_cmd_ = 0, w_cmd_ = 0;
  float ref_l_ = 0, ref_r_ = 0;
  float trim_ = 0;  // > 0: left motor stronger than right (slowed by trim_, right sped up)
  bool has_stop_ = false;
  float s_stop_ = 0, v_end_ = 0;
  bool hold_ = false;
  bool auto_hold_ = false;
  float theta_hold_ = 0;
  float last_pos_ = 0, dpos_ = 0;
  bool have_last_pos_ = false;

  // STRAIGHT
  float s_target_ = 0;
  int dir_sign_ = 1;

  // PIVOT
  float theta0_ = 0, target_ = 0, omega_max_ = 0, alpha_ = 0, capture_after_ = -1, overshoot_ = 0;
  bool capture_seen_off_ = false;
};

}  // namespace mesh
