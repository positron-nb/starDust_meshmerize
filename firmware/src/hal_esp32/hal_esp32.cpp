#ifdef ARDUINO
#include "hal_esp32.h"

#include <Preferences.h>
#include <Wire.h>
#include <driver/gpio.h>

#include "pins.h"

namespace mesh {

namespace {
Preferences g_prefs;
constexpr const char* kNvsNamespace = "meshmerize";
constexpr const char* kNvsKey = "blob";
constexpr uint32_t kPwmMax = (1u << kPwmBits) - 1;
constexpr uint8_t kMpuAddr = 0x68;  // AD0 low

// Slot-disc pulse counters (LM393 modules): one interrupt per edge.
volatile int32_t g_enc_l = 0;
volatile int32_t g_enc_r = 0;
void IRAM_ATTR isrEncL() { g_enc_l = g_enc_l + 1; }
void IRAM_ATTR isrEncR() { g_enc_r = g_enc_r + 1; }

void pwmAttach(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, kPwmFreqHz, kPwmBits);
#else
  ledcSetup(ch, kPwmFreqHz, kPwmBits);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(kMpuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
}  // namespace

void HalEsp32::begin() {
  // Motors off first. STBY (TB6612) has a hardware pull-down for the boot phase.
  pinMode(pins::kStby, OUTPUT);
  digitalWrite(pins::kStby, LOW);
#if MESH_DRIVER == DRIVER_DRV8833
  pwmAttach(pins::kAin1, kPwmChA);
  pwmAttach(pins::kAin2, kPwmChA2);
  pwmAttach(pins::kBin1, kPwmChB);
  pwmAttach(pins::kBin2, kPwmChB2);
  pwmWrite(pins::kAin1, kPwmChA, 0);
  pwmWrite(pins::kAin2, kPwmChA2, 0);
  pwmWrite(pins::kBin1, kPwmChB, 0);
  pwmWrite(pins::kBin2, kPwmChB2, 0);
#else
  pinMode(pins::kAin1, OUTPUT);
  pinMode(pins::kAin2, OUTPUT);
  pinMode(pins::kBin1, OUTPUT);
  pinMode(pins::kBin2, OUTPUT);
  pwmAttach(pins::kPwmA, kPwmChA);
  pwmAttach(pins::kPwmB, kPwmChB);
  pwmWrite(pins::kPwmA, kPwmChA, 0);
  pwmWrite(pins::kPwmB, kPwmChB, 0);
#endif

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // ~0-3.1 V input range
  for (uint8_t p : pins::kSensor) pinMode(p, INPUT);
#if MESH_VBAT_SENSE
  pinMode(pins::kVbatSense, INPUT);
#endif

  pinMode(pins::kButton, INPUT_PULLUP);
  pinMode(pins::kLedRed, OUTPUT);
  digitalWrite(pins::kLedRed, LOW);
  // Weakest output drive (~5 mA class): limits the current if the LED is
  // wired without a series resistor. A 330R resistor is still better.
  gpio_set_drive_capability((gpio_num_t)pins::kLedRed, GPIO_DRIVE_CAP_0);
  pinMode(pins::kLedStatus, OUTPUT);
  digitalWrite(pins::kLedStatus, LOW);

  // Optional slot-disc encoders: count every edge (2 x slots per revolution).
  pinMode(pins::kEncL, INPUT_PULLUP);
  pinMode(pins::kEncR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pins::kEncL), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::kEncR), isrEncR, CHANGE);

  setupGyro();
}

void HalEsp32::setupGyro() {
  Wire.begin(pins::kSda, pins::kScl, 400000);
  Wire.beginTransmission(kMpuAddr);
  Wire.write(0x75);  // WHO_AM_I
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)kMpuAddr, 1) != 1) {
    gyro_ok_ = false;
    return;
  }
  uint8_t who = Wire.read();
  // MPU6050 = 0x68; common clones/relatives (MPU6500 0x70, MPU9250 0x71) share the gyro registers
  gyro_ok_ = (who == 0x68 || who == 0x70 || who == 0x71 || who == 0x72 || who == 0x98);
  if (!gyro_ok_) return;
  mpuWrite(0x6B, 0x01);  // PWR_MGMT_1: wake up, PLL with X gyro clock
  mpuWrite(0x1A, 0x03);  // CONFIG: DLPF ~42 Hz
  mpuWrite(0x1B, 0x08);  // GYRO_CONFIG: +-500 deg/s (65.5 LSB per deg/s)
  delay(50);
}

bool HalEsp32::readGyro(float& dps) {
  Wire.beginTransmission(kMpuAddr);
  Wire.write(0x47);  // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)kMpuAddr, 2) != 2) return false;
  int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
  dps = raw / 65.5f;
  return true;
}

void HalEsp32::read(HalInputs& in) {
  in.t_us = (uint32_t)micros();
  // Each channel is converted twice and the first result discarded: high-
  // impedance sensor outputs can carry charge over from the previous channel.
  for (int i = 0; i < 7; ++i) {
    (void)analogRead(pins::kSensor[i]);
    in.raw[i] = (uint16_t)analogRead(pins::kSensor[i]);
  }
  for (int i = 7; i < kMaxSensors; ++i) in.raw[i] = 0;
  noInterrupts();
  in.enc_left = g_enc_l;
  in.enc_right = g_enc_r;
  interrupts();
#if MESH_VBAT_SENSE
  float v = analogReadMilliVolts(pins::kVbatSense) * 0.001f * kVbatDividerRatio;
  vbat_filt_ = vbat_filt_ <= 0.1f ? v : vbat_filt_ + 0.02f * (v - vbat_filt_);
  in.vbat = vbat_filt_;
#else
  in.vbat = 0;  // not measured: the core uses vbat_nominal_v
#endif
  in.button = digitalRead(pins::kButton) == LOW ? 1 : 0;
  in.gyro_ok = 0;
  in.gyro_z_dps = 0;
  if (gyro_ok_) {
    float dps;
    if (readGyro(dps)) {
      in.gyro_z_dps = dps;
      in.gyro_ok = 1;
      gyro_fail_ = 0;
    } else if (++gyro_fail_ > 20) {
      gyro_ok_ = false;  // wire came loose: core falls back to the wheel/model estimate
    }
  }
}

void HalEsp32::driveMotor(float duty, bool left) {
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  const uint8_t in1 = left ? pins::kAin1 : pins::kBin1;
  const uint8_t in2 = left ? pins::kAin2 : pins::kBin2;
  const uint32_t d = (uint32_t)((duty < 0 ? -duty : duty) * kPwmMax + 0.5f);
#if MESH_DRIVER == DRIVER_DRV8833
  const uint8_t c1 = left ? kPwmChA : kPwmChB;
  const uint8_t c2 = left ? kPwmChA2 : kPwmChB2;
  if (duty > 0.001f) {         // forward: PWM on IN1, IN2 low (fast decay)
    pwmWrite(in1, c1, d);
    pwmWrite(in2, c2, 0);
  } else if (duty < -0.001f) {
    pwmWrite(in1, c1, 0);
    pwmWrite(in2, c2, d);
  } else {                     // brake: both high
    pwmWrite(in1, c1, kPwmMax);
    pwmWrite(in2, c2, kPwmMax);
  }
#else
  const uint8_t pwm = left ? pins::kPwmA : pins::kPwmB;
  const uint8_t ch = left ? kPwmChA : kPwmChB;
  if (duty > 0.001f) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    pwmWrite(pwm, ch, d);
  } else if (duty < -0.001f) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    pwmWrite(pwm, ch, d);
  } else {  // brake: both inputs high, enable on (TB6612 and L298N both short the motor)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    pwmWrite(pwm, ch, kPwmMax);
  }
#endif
}

void HalEsp32::write(const HalOutputs& out) {
#if MESH_DRIVER == DRIVER_TB6612
  digitalWrite(pins::kStby, out.motors_enable ? HIGH : LOW);
#endif
  driveMotor(out.motors_enable ? out.duty_left : 0.0f, true);
  driveMotor(out.motors_enable ? out.duty_right : 0.0f, false);
  digitalWrite(pins::kLedRed, out.led_red ? HIGH : LOW);
  digitalWrite(pins::kLedStatus, out.led_status ? HIGH : LOW);
}

void HalEsp32::stopMotors() {
  driveMotor(0, true);
  driveMotor(0, false);
}

size_t HalEsp32::loadStorage(uint8_t* buf, size_t cap) {
  if (!g_prefs.begin(kNvsNamespace, true)) return 0;
  size_t n = g_prefs.getBytesLength(kNvsKey);
  if (n == 0 || n > cap) {
    g_prefs.end();
    return 0;
  }
  size_t got = g_prefs.getBytes(kNvsKey, buf, n);
  g_prefs.end();
  return got == n ? n : 0;
}

bool HalEsp32::saveStorage(const uint8_t* buf, size_t len) {
  if (!g_prefs.begin(kNvsNamespace, false)) return false;
  size_t w = g_prefs.putBytes(kNvsKey, buf, len);
  g_prefs.end();
  return w == len;
}

}  // namespace mesh
#endif  // ARDUINO
