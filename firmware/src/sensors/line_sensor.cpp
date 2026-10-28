#include "line_sensor.h"

#include <string.h>

#include "../util/util.h"

namespace mesh {

void LineSensor::init(const RobotConfig* cfg) {
  cfg_ = cfg;
  beginCalibration();
  cal_.valid = 0;
}

void LineSensor::beginCalibration() {
  for (int i = 0; i < kMaxSensors; ++i) {
    cal_.min_raw[i] = 0xFFFF;
    cal_.max_raw[i] = 0;
    cal_sum_[i] = 0;
  }
  cal_count_ = 0;
  cal_.valid = 0;
}

void LineSensor::sampleCalibration(const uint16_t* raw) {
  for (int i = 0; i < channels(); ++i) {
    if (raw[i] < cal_.min_raw[i]) cal_.min_raw[i] = raw[i];
    if (raw[i] > cal_.max_raw[i]) cal_.max_raw[i] = raw[i];
    cal_sum_[i] += raw[i];
  }
  ++cal_count_;
}

bool LineSensor::finishCalibration() {
  bad_mask_ = 0;
  cal_.dark_high_mask = 0;
  for (int i = 0; i < channels(); ++i) {
    if (cal_.max_raw[i] < cal_.min_raw[i] ||
        (float)(cal_.max_raw[i] - cal_.min_raw[i]) < cfg_->cal_min_span_raw) {
      bad_mask_ |= (uint16_t)(1u << i);
      continue;
    }
    // Polarity: the sweep spends most of its time over the black floor, so
    // the reading the channel sits at MOST of the time is "black". This works
    // for analog arrays (QTR: high over black) and for digital comparator
    // modules (TCRT5000: usually low over white) without configuration.
    bool dark_high = cfg_->sensor_dark_high != 0;
    if (cfg_->sensor_auto_polarity && cal_count_ > 0) {
      float mean = cal_sum_[i] / (float)cal_count_;
      float mid = 0.5f * ((float)cal_.min_raw[i] + (float)cal_.max_raw[i]);
      dark_high = mean > mid;
    }
    if (dark_high) cal_.dark_high_mask |= (uint16_t)(1u << i);
  }
  cal_.valid = bad_mask_ == 0 ? 1 : 0;
  if (bad_mask_) Log::printf("CAL bad channels mask=0x%04x", bad_mask_);
  else Log::printf("CAL ok: polarity mask (1 = high over black) 0x%04x", cal_.dark_high_mask);
  return cal_.valid != 0;
}

void LineSensor::setCalibration(const SensorCalibration& c) {
  cal_ = c;
  bad_mask_ = 0;
}

bool LineSensor::darkHigh(int i) const {
  if (cal_.valid) return (cal_.dark_high_mask >> i) & 1u;
  return cfg_->sensor_dark_high != 0;
}

float LineSensor::sensorY(int i) const {
  const int n = cfg_->num_sensors;
  if (i >= n) {  // wing sensors: n = left, n+1 = right
    return (i == n ? -cfg_->wing_lateral_mm : cfg_->wing_lateral_mm) + cfg_->sensor_lateral_offset_mm;
  }
  float y = ((float)i - (n - 1) * 0.5f) * cfg_->sensor_pitch_mm;
  if (cfg_->sensor_flip) y = -y;
  return y + cfg_->sensor_lateral_offset_mm;
}

void LineSensor::process(const uint16_t* raw, LineFeatures& f) {
  const int n = cfg_->num_sensors;  // line array only; wings handled separately
  const int nc = channels();
  const float on = cfg_->sensor_on_norm;
  f.on_mask = 0;
  f.n_on = 0;
  bool wing_on[2] = {false, false};
  for (int i = 0; i < nc; ++i) {
    float lo = cal_.min_raw[i], hi = cal_.max_raw[i];
    float span = hi - lo;
    if (!cal_.valid || span < 1.0f) {
      lo = 0;  // uncalibrated fallback: assume full 12-bit range
      span = 4095.0f;
    }
    float v = clampf(((float)raw[i] - lo) / span, 0.0f, 1.0f);
    if (darkHigh(i)) v = 1.0f - v;  // normalise to 1000 = white
    f.val[i] = (uint16_t)(v * 1000.0f + 0.5f);
    if (i >= n) {
      wing_on[i - n] = f.val[i] >= on;
      continue;
    }
    if (f.val[i] >= on) {
      f.on_mask |= (uint16_t)(1u << i);
      ++f.n_on;
    }
  }
  const bool wings = nc > n;
  f.wing_left = wings && wing_on[0];
  f.wing_right = wings && wing_on[1];
  f.line_seen = f.n_on > 0;
  // A one-sided cross line seen while the robot is off-centre lights n-1
  // array channels, so "both sides" needs every channel (and both wings when
  // fitted). The goal test tolerates one weak array channel.
  f.all_white = f.n_on >= n && (!wings || (f.wing_left && f.wing_right));
  f.goal_white = f.n_on >= (n >= 6 ? n - 1 : n) && (!wings || (f.wing_left && f.wing_right));

  // Find contiguous runs of "on" channels in index order and pick the one
  // whose centre is nearest the previous line position (keeps tracking the
  // same line when a second line enters the field of view).
  int best_a = -1, best_b = -1;
  float best_d = 1e9f;
  int i = 0;
  while (i < n) {
    if (!(f.on_mask & (1u << i))) { ++i; continue; }
    int a = i;
    while (i < n && (f.on_mask & (1u << i))) ++i;
    int b = i - 1;
    float c = 0.5f * (sensorY(a) + sensorY(b));
    float d = absf(c - last_pos_);
    if (d < best_d) { best_d = d; best_a = a; best_b = b; }
  }

  if (best_a >= 0) {
    // Weighted centroid over the run plus one neighbour on each side (the
    // analogue edge values improve resolution beyond the sensor pitch).
    int a = best_a > 0 ? best_a - 1 : best_a;
    int b = best_b < n - 1 ? best_b + 1 : best_b;
    float sw = 0, swy = 0;
    for (int k = a; k <= b; ++k) {
      float w = f.val[k] > cfg_->sensor_noise_norm ? (float)f.val[k] : 0.0f;
      sw += w;
      swy += w * sensorY(k);
    }
    f.pos_mm = sw > 0 ? swy / sw : 0.5f * (sensorY(best_a) + sensorY(best_b));
    float ya = sensorY(best_a), yb = sensorY(best_b);
    f.run_left_mm = minf(ya, yb);
    f.run_right_mm = maxf(ya, yb);
    f.run_len = (uint8_t)(best_b - best_a + 1);
    last_pos_ = f.pos_mm;
    // Branch flags: the main run touches an edge channel, is wider than a
    // single line can be, AND that edge channel lies beyond the line we were
    // tracking. Without the last condition an off-centre robot whose line
    // already lights one edge would report a branch on both sides when a
    // one-sided branch fills the rest of the array.
    bool wide = f.run_len >= cfg_->branch_min_sensors;
    int left_idx = cfg_->sensor_flip ? n - 1 : 0;
    int right_idx = cfg_->sensor_flip ? 0 : n - 1;
    const float side = cfg_->branch_side_margin_mm;
    bool touches_left = (f.on_mask & (1u << left_idx)) && (best_a == left_idx || best_b == left_idx) &&
                        sensorY(left_idx) < clean_pos_ - side;
    bool touches_right = (f.on_mask & (1u << right_idx)) && (best_a == right_idx || best_b == right_idx) &&
                         sensorY(right_idx) > clean_pos_ + side;
    f.left_branch = wide && touches_left;
    f.right_branch = wide && touches_right;
    // Slow average (~25 mm of travel): the first edge of a crossbar can pull
    // the tracked position towards the branch for a few samples.
    if (!wide && !(wings && (wing_on[0] || wing_on[1]))) clean_pos_ += 0.03f * (f.pos_mm - clean_pos_);
    // "Centre" must include the channels next to the middle one: with a
    // 15 mm pitch a slightly tilted robot sees its line only on those.
    const float cw = maxf(12.0f, 1.2f * cfg_->sensor_pitch_mm);
    f.center_on = f.run_left_mm <= cw && f.run_right_mm >= -cw;
  } else {
    f.pos_mm = last_pos_;
    f.run_left_mm = f.run_right_mm = 0;
    f.run_len = 0;
    f.left_branch = f.right_branch = false;
    f.center_on = false;
  }
  // Wing sensors sit outside the array, beyond where a tracked 30 mm line
  // can reach: if one sees white, a line leaves on that side.
  if (f.wing_left && sensorY(n) < clean_pos_ - cfg_->branch_side_margin_mm) f.left_branch = true;
  if (f.wing_right && sensorY(n + 1) > clean_pos_ + cfg_->branch_side_margin_mm) f.right_branch = true;
  if (f.all_white) f.left_branch = f.right_branch = true;
}

}  // namespace mesh
