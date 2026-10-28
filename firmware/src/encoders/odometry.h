// odometry.h - distance and turn angle from whatever the robot has:
//
//   encoder_mode 0  quadrature encoders (signed counts, e.g. N20 + magnetic encoder)
//   encoder_mode 1  slot-disc encoders (LM393 photo-interrupter): pulses only
//                   count up, so the direction comes from the commanded speed
//   encoder_mode 2  no encoders: distance = integral of a first-order model of
//                   the commanded wheel speed (needs ff_mm_s_per_v calibrated)
//
// Turn angle (theta) comes from the MPU6050 gyro whenever use_gyro=1 and the
// HAL reports a valid reading; otherwise from the wheel difference. The gyro
// bias is re-estimated automatically whenever the robot is commanded to stand
// still.
//
// s     : distance travelled by the axle centre along the path (mm, signed).
// theta : accumulated rotation (rad, CCW positive).
// The map heading is NOT taken from theta: lines meet at 90 degrees, so the
// heading is kept as a discrete N/E/S/W index and theta is only used to
// measure individual turns. This removes heading drift from the map.
#pragma once
#include <stdint.h>

#include "config.h"

namespace mesh {

class Odometry {
 public:
  void init(const RobotConfig* cfg);
  void reset(int32_t enc_left, int32_t enc_right);
  // ref_l/ref_r: wheel speeds commanded in the previous step (mm/s).
  void update(int32_t enc_left, int32_t enc_right, float dt_s, float gyro_dps, bool gyro_ok, float ref_l,
              float ref_r);

  float s() const { return s_; }
  float theta() const { return theta_; }
  float leftMm() const { return left_mm_; }
  float rightMm() const { return right_mm_; }
  float vLeft() const { return v_left_; }    // mm/s, filtered (model speed in mode 2)
  float vRight() const { return v_right_; }
  float v() const { return 0.5f * (v_left_ + v_right_); }
  float omega() const { return omega_; }     // rad/s
  float mmPerCount(bool left) const;
  bool hasWheelFeedback() const { return cfg_->encoder_mode != 2; }
  bool gyroActive() const { return gyro_active_; }
  float gyroBias() const { return gyro_bias_; }
  float gyroTheta() const { return gyro_theta_; }
  float wheelTheta() const { return wheel_theta_; }

 private:
  const RobotConfig* cfg_ = nullptr;
  int32_t last_l_ = 0, last_r_ = 0;
  bool first_ = true;
  float s_ = 0, theta_ = 0, left_mm_ = 0, right_mm_ = 0;
  float v_left_ = 0, v_right_ = 0, omega_ = 0;
  float model_l_ = 0, model_r_ = 0;  // mode 2 wheel speed model
  float gyro_bias_ = 0, gyro_theta_ = 0, wheel_theta_ = 0;
  float still_ms_ = 0;
  bool gyro_active_ = false;
  float slot_dir_l_ = 1, slot_dir_r_ = 1;
};

}  // namespace mesh
