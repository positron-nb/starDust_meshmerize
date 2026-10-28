#include "odometry.h"

#include "../util/util.h"

namespace mesh {

void Odometry::init(const RobotConfig* cfg) { cfg_ = cfg; }

float Odometry::mmPerCount(bool left) const {
  float scale = left ? cfg_->wheel_scale_left : cfg_->wheel_scale_right;
  return kPi * cfg_->wheel_diameter_mm * scale / cfg_->counts_per_wheel_rev;
}

void Odometry::reset(int32_t enc_left, int32_t enc_right) {
  last_l_ = enc_left;
  last_r_ = enc_right;
  first_ = false;
  s_ = theta_ = left_mm_ = right_mm_ = 0;
  v_left_ = v_right_ = omega_ = 0;
  model_l_ = model_r_ = 0;
  gyro_theta_ = wheel_theta_ = 0;
}

void Odometry::update(int32_t enc_left, int32_t enc_right, float dt_s, float gyro_dps, bool gyro_ok, float ref_l,
                      float ref_r) {
  if (first_) reset(enc_left, enc_right);
  int32_t dl_c = enc_left - last_l_;
  int32_t dr_c = enc_right - last_r_;
  last_l_ = enc_left;
  last_r_ = enc_right;
  float dl = 0, dr = 0;
  const int mode = cfg_->encoder_mode;
  if (mode == 0) {
    if (cfg_->left_enc_invert) dl_c = -dl_c;
    if (cfg_->right_enc_invert) dr_c = -dr_c;
    dl = dl_c * mmPerCount(true);
    dr = dr_c * mmPerCount(false);
  } else if (mode == 1) {
    // Slot discs give unsigned pulses: the wheel turns the way it was driven
    // (keep the last direction while the command crosses zero).
    if (absf(ref_l) > 5.0f) slot_dir_l_ = signf(ref_l);
    if (absf(ref_r) > 5.0f) slot_dir_r_ = signf(ref_r);
    dl = (dl_c < 0 ? -dl_c : dl_c) * mmPerCount(true) * slot_dir_l_;
    dr = (dr_c < 0 ? -dr_c : dr_c) * mmPerCount(false) * slot_dir_r_;
  } else {
    // No encoders: first-order model of the commanded wheel speeds.
    if (dt_s > 0) {
      float a = clampf(dt_s / (cfg_->motor_tau_ms * 0.001f), 0.0f, 1.0f);
      model_l_ += a * (ref_l - model_l_);
      model_r_ += a * (ref_r - model_r_);
    }
    dl = model_l_ * dt_s;
    dr = model_r_ * dt_s;
  }
  left_mm_ += dl;
  right_mm_ += dr;
  s_ += 0.5f * (dl + dr);
  float dth_wheel = (dr - dl) / cfg_->track_width_mm;
  wheel_theta_ += dth_wheel;

  // Gyro: bias tracking while commanded still, integration otherwise.
  gyro_active_ = cfg_->use_gyro && gyro_ok;
  float dth = dth_wheel;
  if (gyro_active_) {
    float g = gyro_dps * cfg_->gyro_sign;
    bool still = absf(ref_l) < 1.0f && absf(ref_r) < 1.0f;
    still_ms_ = still ? still_ms_ + dt_s * 1000.0f : 0.0f;
    if (still_ms_ > 300.0f) gyro_bias_ += clampf(dt_s / 1.0f, 0.0f, 1.0f) * (g - gyro_bias_);  // 1 s average
    float rate = (still_ms_ > 300.0f) ? 0.0f : (g - gyro_bias_) * kDeg;
    dth = rate * dt_s;
    gyro_theta_ += dth;
  }
  theta_ += dth;

  if (dt_s > 0) {
    if (mode == 2) {
      v_left_ = model_l_;
      v_right_ = model_r_;
    } else {
      // Low-pass the quantised wheel speeds (slot discs are coarse: ~5 mm/count).
      const float tau = mode == 1 ? 0.040f : 0.010f;
      const float a = clampf(dt_s / tau, 0.0f, 1.0f);
      v_left_ += a * (dl / dt_s - v_left_);
      v_right_ += a * (dr / dt_s - v_right_);
    }
    omega_ = gyro_active_ ? dth / dt_s : (v_right_ - v_left_) / cfg_->track_width_mm;
  }
}

}  // namespace mesh
