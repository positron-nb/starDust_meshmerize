// hal_esp32.h - the only code that touches ESP32 peripherals.
// Converts between the physical robot and the HalInputs/HalOutputs structs
// consumed by mesh::Robot (which is identical in the simulator).
#pragma once
#ifdef ARDUINO
#include <Arduino.h>

#include "robot_types.h"

namespace mesh {

class HalEsp32 {
 public:
  void begin();
  void read(HalInputs& in);
  void write(const HalOutputs& out);
  void stopMotors();
  bool gyroPresent() const { return gyro_ok_; }
  // Non-volatile storage: one blob in the NVS partition (Preferences API).
  size_t loadStorage(uint8_t* buf, size_t cap);
  bool saveStorage(const uint8_t* buf, size_t len);

 private:
  void setupGyro();
  bool readGyro(float& dps);
  void driveMotor(float duty, bool left);
  float vbat_filt_ = 0;
  bool gyro_ok_ = false;
  uint8_t gyro_fail_ = 0;
};

}  // namespace mesh
#endif
