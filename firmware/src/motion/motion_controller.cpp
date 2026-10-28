#include "motion_controller.h"

#include <math.h>

#include "../util/util.h"

namespace mesh {

// ------------------------------------------------------------------ wheels --
float WheelSpeedController::one(float vref, float vmeas, float vbat, float dt, float& integ) {
  if (absf(vref) < 1.0f) {
    // Commanded stop: actively brake towards zero speed, no integrator wind-up.
    integ = 0;
    // Slot discs cannot see direction: braking against a speed whose sign is
    // only assumed runs away once the wheel reverses. Zero duty = the driver
    // shorts the motor, which brakes it anyway.
    if (cfg_->encoder_mode == 1) return 0.0f;
    float v = -cfg_->wheel_kp_v_per_mm_s * vmeas * 2.0f;
    return clampf(v / vbat, -0.3f, 0.3f);
  }
  float v_ff = vref / cfg_->ff_mm_s_per_v + signf(vref) * cfg_->ff_static_v;
  float volts = v_ff;
  // Closed loop only with quadrature encoders. Slot discs (~5 mm per count)
  // give too coarse and laggy a speed for a 1 kHz loop: they are used for
  // distance only, and motor balance comes from the learned trim.
  if (cfg_->encoder_mode == 0) {
    float err = vref - vmeas;
    integ = clampf(integ + cfg_->wheel_ki_v_per_mm * err * dt, -2.0f, 2.0f);
    volts += cfg_->wheel_kp_v_per_mm_s * err + integ;
  }
  float dmax = minf(1.0f, cfg_->motor_max_avg_v / vbat);  // never exceed motor rating
  return clampf(volts / vbat, -dmax, dmax);
}

void WheelSpeedController::update(float vref_l, float vref_r, float vmeas_l, float vmeas_r, float vbat,
                                  float dt, float& duty_l, float& duty_r) {
  if (vbat < 3.0f) vbat = cfg_->vbat_nominal_v;
  duty_l = one(vref_l, vmeas_l, vbat, dt, i_l_);
  duty_r = one(vref_r, vmeas_r, vbat, dt, i_r_);
}

// ------------------------------------------------------------------ motion --
void MotionController::init(const RobotConfig* cfg, const Odometry* odo) {
  cfg_ = cfg;
  odo_ = odo;
  wheels_.init(cfg);
  idle();
}

void MotionController::idle() {
  mode_ = MotionMode::IDLE;
  v_cmd_ = w_cmd_ = 0;
  done_ = true;
  failed_ = false;
  wheels_.reset();
}

void MotionController::brake(float decel) {
  mode_ = MotionMode::BRAKE;
  decel_ = decel;
  done_ = false;
  failed_ = false;
}

void MotionController::follow(float v_cruise, float accel, float decel) {
  if (mode_ != MotionMode::FOLLOW) {
    have_last_pos_ = false;
    hold_ = false;
    auto_hold_ = false;
    has_stop_ = false;
  }
  mode_ = MotionMode::FOLLOW;
  v_cruise_ = v_cruise;
  accel_ = accel;
  decel_ = decel;
  done_ = false;
  failed_ = false;
}

void MotionController::holdHeading(bool on) {
  if (on && !hold_) theta_hold_ = odo_->theta();
  if (!on && hold_) have_last_pos_ = false;  // restart the derivative after a hold
  hold_ = on;
}

void MotionController::straight(float dist, float v, float accel, float decel) {
  mode_ = MotionMode::STRAIGHT;
  s_target_ = odo_->s() + dist;
  dir_sign_ = dist >= 0 ? 1 : -1;
  v_cruise_ = absf(v);
  accel_ = accel;
  decel_ = decel;
  theta_hold_ = odo_->theta();
  // keep current v_cmd_ for a smooth hand-over
  done_ = false;
  failed_ = false;
}

void MotionController::pivot(float angle, float omega_max, float alpha, float capture_after, float overshoot) {
  mode_ = MotionMode::PIVOT;
  theta0_ = odo_->theta();
  target_ = angle;
  omega_max_ = omega_max;
  alpha_ = alpha;
  capture_after_ = capture_after;
  overshoot_ = overshoot;
  capture_seen_off_ = false;
  v_cmd_ = 0;
  w_cmd_ = 0;
  done_ = false;
  failed_ = false;
}

float MotionController::pivotRotated() const { return odo_->theta() - theta0_; }

void MotionController::wheelsFrom(float v, float w, float& vl, float& vr) const {
  float half = 0.5f * cfg_->track_width_mm;
  vl = v - w * half;
  vr = v + w * half;
}

static float rampTo(float cur, float target, float up, float down, float dt) {
  if (target > cur) return minf(target, cur + up * dt);
  return maxf(target, cur - down * dt);
}

void MotionController::update(const LineFeatures& f, float dt, float vbat, HalOutputs& out) {
  float vl_ref = 0, vr_ref = 0;
  switch (mode_) {
    case MotionMode::IDLE:
      out.duty_left = out.duty_right = 0;
      ref_l_ = ref_r_ = 0;
      wheels_.reset();
      return;

    case MotionMode::BRAKE: {
      v_cmd_ = rampTo(v_cmd_, 0, decel_, decel_, dt);
      w_cmd_ = 0;
      wheelsFrom(v_cmd_, 0, vl_ref, vr_ref);
      if (absf(v_cmd_) < 1.0f && absf(odo_->v()) < 25.0f) done_ = true;
      break;
    }

    case MotionMode::FOLLOW: {
      float s = odo_->s();
      float v_target = v_cruise_;
      if (has_stop_) {
        float rem = s_stop_ - s;
        float v_allow = rem > 0 ? sqrtf(v_end_ * v_end_ + 2.0f * decel_ * rem) : v_end_;
        v_target = minf(v_target, v_allow);
      }
      v_cmd_ = rampTo(v_cmd_, v_target, accel_, decel_ * 1.2f, dt);
      if (!hold_ && !f.line_seen && !auto_hold_) {
        theta_hold_ = odo_->theta();  // line gap: remember heading
        auto_hold_ = true;
      } else if (f.line_seen) {
        auto_hold_ = false;
      }
      if (hold_ || auto_hold_) {
        // Junction crossing (commanded) or line gap: hold the encoder heading.
        w_cmd_ = clampf(8.0f * (theta_hold_ - odo_->theta()), -3.0f, 3.0f);
        have_last_pos_ = false;
      } else {
        float pos = f.pos_mm;
        if (have_last_pos_ && dt > 0) {
          float raw_d = (pos - last_pos_) / dt;
          dpos_ += clampf(dt / 0.004f, 0.0f, 1.0f) * (raw_d - dpos_);
        } else {
          dpos_ = 0;
        }
        last_pos_ = pos;
        have_last_pos_ = true;
        float kp = cfg_->line_kp + cfg_->line_kp_per_m_s * absf(v_cmd_) * 0.001f;
        // line to the right (pos > 0) -> turn clockwise (negative omega)
        w_cmd_ = clampf(-(kp * pos + cfg_->line_kd * dpos_), -cfg_->max_steer_rad_s, cfg_->max_steer_rad_s);
        // No wheel-speed loop (no or slot encoders): nothing evens out two unequal motors, so the line
        // follower ends up steering constantly to one side on a straight line.
        // That steady steering is the gain imbalance; learn it as a trim.
        // Model: true yaw = w_cmd + v (trim - d) / half, d = gain imbalance,
        // so on a straight line trim error = w_cmd * half / v.
        if (cfg_->encoder_mode != 0 && f.line_seen && f.center_on && !f.left_branch && !f.right_branch &&
            v_cmd_ > 150.0f) {
          float half = 0.5f * cfg_->track_width_mm;
          trim_ = clampf(trim_ + dt / cfg_->motor_trim_tau_s * w_cmd_ * half / v_cmd_, -0.3f, 0.3f);
        }
      }
      wheelsFrom(v_cmd_, w_cmd_, vl_ref, vr_ref);
      break;
    }

    case MotionMode::STRAIGHT: {
      float rem = (s_target_ - odo_->s()) * dir_sign_;
      float v_allow = rem > 0 ? sqrtf(2.0f * decel_ * rem) : 0;
      float target = minf(v_cruise_, maxf(v_allow, rem > 1.0f ? 40.0f : 0.0f)) * dir_sign_;
      v_cmd_ = rampTo(v_cmd_, target, accel_, decel_ * 1.2f, dt);
      w_cmd_ = clampf(8.0f * (theta_hold_ - odo_->theta()), -3.0f, 3.0f);
      if (rem <= 1.0f) {
        v_cmd_ = 0;
        w_cmd_ = 0;
        done_ = true;
      }
      wheelsFrom(v_cmd_, w_cmd_, vl_ref, vr_ref);
      break;
    }

    case MotionMode::PIVOT: {
      float sgn = target_ >= 0 ? 1.0f : -1.0f;
      float rot = (odo_->theta() - theta0_) * sgn;  // progress, positive
      float rem = absf(target_) - rot;
      bool capturing = capture_after_ >= 0 && rot >= capture_after_;
      if (capturing) {
        // Require the array to have been off the old line once (avoids
        // instantly "capturing" the line we are leaving on a U-turn).
        if (!f.center_on) capture_seen_off_ = true;
        // A line wider than the pitch makes a digital array jump between
        // +-pitch/2 readings, so the tolerance must exceed half the pitch.
        float tol = maxf(cfg_->turn_capture_tol_mm, 0.6f * cfg_->sensor_pitch_mm);
        if (f.center_on && absf(f.pos_mm) <= tol &&
            (capture_seen_off_ || rot > absf(target_) * 0.6f)) {
          done_ = true;
        } else if (rem < -overshoot_) {
          failed_ = true;
          done_ = true;
        }
      } else if (capture_after_ < 0 && rem <= 0.01f) {
        done_ = true;
      }
      float w_mag;
      if (done_) {
        w_mag = 0;
      } else {
        float w_prof = rem > 0 ? sqrtf(2.0f * alpha_ * rem) : 0;
        float w_min = capturing ? 1.6f : 0.8f;  // keep creeping until the line is captured
        w_mag = clampf(w_prof, w_min, omega_max_);
        w_mag = minf(w_mag, absf(w_cmd_) + alpha_ * dt);
      }
      w_cmd_ = sgn * w_mag;
      v_cmd_ = 0;
      wheelsFrom(0, w_cmd_, vl_ref, vr_ref);
      if (done_) {
        v_cmd_ = 0;
        w_cmd_ = 0;
      }
      break;
    }
  }
  ref_l_ = vl_ref;  // intended wheel speeds (the no-encoder odometry model uses these)
  ref_r_ = vr_ref;
  if (cfg_->encoder_mode != 0) {
    vl_ref *= 1.0f - trim_;
    vr_ref *= 1.0f + trim_;
  }
  float dl, dr;
  wheels_.update(vl_ref, vr_ref, odo_->vLeft(), odo_->vRight(), vbat, dt, dl, dr);
  if (cfg_->left_motor_invert) dl = -dl;
  if (cfg_->right_motor_invert) dr = -dr;
  out.duty_left = dl;
  out.duty_right = dr;
}

}  // namespace mesh
