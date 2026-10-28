// line_sensor.h - reflectance array calibration, normalisation and feature
// extraction (line position, branch flags, all-white flag).
//
// Convention: normalised value 1000 = white vinyl line, 0 = black floor.
// Lateral positions are in mm, positive = to the robot's RIGHT.
#pragma once
#include <stdint.h>

#include "config.h"
#include "robot_types.h"

namespace mesh {

struct SensorCalibration {
  uint16_t min_raw[kMaxSensors];
  uint16_t max_raw[kMaxSensors];
  uint8_t valid;            // 1 once a calibration sweep has completed successfully
  uint8_t pad;
  uint16_t dark_high_mask;  // bit i = channel i reads HIGH over the black floor (learned)
};

struct LineFeatures {
  uint16_t val[kMaxSensors];  // normalised 0..1000 (1000 = white)
  uint16_t on_mask;           // bit i = channel i sees white
  uint8_t n_on;               // number of channels on
  bool line_seen;             // any channel on
  bool center_on;             // the run nearest the centre covers |y| < 12 mm
  float pos_mm;               // line centre (main white run), + = right
  float run_left_mm;          // extent of the main run
  float run_right_mm;
  uint8_t run_len;            // channels in the main run
  bool left_branch;           // main run reaches the left edge and is wide
  bool right_branch;
  bool all_white;             // EVERY channel white (cross line on both sides)
  bool goal_white;            // all but at most one channel white (end-box test,
                              // tolerant to one weak channel on a dull box)
  bool wing_left, wing_right; // wing sensors see white (false when there are no wings)
};

class LineSensor {
 public:
  void init(const RobotConfig* cfg);
  // Calibration: call begin, then sample() while sweeping the array across
  // the line and the floor, then finish() (returns false if a channel never
  // saw enough contrast - see badMask()).
  void beginCalibration();
  void sampleCalibration(const uint16_t* raw);
  bool finishCalibration();
  uint16_t badMask() const { return bad_mask_; }
  const SensorCalibration& calibration() const { return cal_; }
  void setCalibration(const SensorCalibration& c);
  bool calibrated() const { return cal_.valid != 0; }

  // Normalise and extract features from one raw sample.
  void process(const uint16_t* raw, LineFeatures& f);
  float sensorY(int i) const;  // lateral position of channel i (array or wing)
  int channels() const { return cfg_->num_sensors + (cfg_->num_wings == 2 ? 2 : 0); }
  void resetTracking(float y_mm = 0.0f) { last_pos_ = clean_pos_ = y_mm; }

 private:
  bool darkHigh(int i) const;
  const RobotConfig* cfg_ = nullptr;
  SensorCalibration cal_{};
  uint16_t bad_mask_ = 0;
  float last_pos_ = 0.0f;
  float clean_pos_ = 0.0f;  // line position on the last sample without a branch pattern
  float cal_sum_[kMaxSensors] = {};  // for polarity learning (not persisted)
  uint32_t cal_count_ = 0;
};

}  // namespace mesh
