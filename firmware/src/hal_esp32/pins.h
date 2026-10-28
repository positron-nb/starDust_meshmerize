// pins.h - ESP32 DevKit V1 (30-pin, ESP32-WROOM-32) pin assignment for the
// PURCHASED hardware: 5-channel digital IR array (D1-D5) + 2 FC-51 IR "wing"
// sensors, BO motors on an L298N, MPU6050, 4S 18650 pack via LM2596 buck.
// Every pin used here also exists on the 38-pin DevKitC.
//
// THIS FILE, hardware/pin_mapping.csv and docs/wiring_and_pinout.md must
// agree (simulator/tests/test_consistency.py checks it). Rules used:
//   * Every IR sensor goes to an ADC-capable pin, so analog AND digital
//     (0/1 comparator) modules both work: digital outputs simply read as
//     0 or 4095. ADC2 pins (GPIO25/26) are usable because Wi-Fi is never started.
//   * MPU6050 on the ESP32's default I2C pins (GPIO21 SDA, GPIO22 SCL).
//   * GPIO6-11 (flash) and GPIO12 (boot strap) are never used; GPIO16/17
//     exist on WROOM modules only (not WROVER).
//   * GPIO5/14/15 toggle during boot: motor-driver STBY/enable lines have a
//     pull-down so the motors stay off until firmware runs.
#pragma once
#ifdef ARDUINO
#include <Arduino.h>

// ---- choose the motor driver you actually have ----------------------------
#define DRIVER_TB6612 1   // PWMA/AIN1/AIN2, PWMB/BIN1/BIN2, STBY
#define DRIVER_L298N 2    // ENA/IN1/IN2, ENB/IN3/IN4 (remove the ENA/ENB jumpers), no STBY
#define DRIVER_DRV8833 3  // DRV8833 or MX1508: IN1..IN4 only, PWM on the IN pins
#ifndef MESH_DRIVER
#define MESH_DRIVER DRIVER_L298N
#endif

namespace pins {
// IR sensors, left -> right on the 5-channel array, then left wing, right wing
constexpr uint8_t kSensor[7] = {36, 39, 34, 35, 32, /*wings*/ 33, 25};
constexpr uint8_t kVbatSense = 26;  // optional 47k (top) / 10k (bottom) divider, see MESH_VBAT_SENSE

// motor driver (names for TB6612; L298N: PWMA=ENA, AIN1/2=IN1/2, PWMB=ENB, BIN1/2=IN3/4;
// DRV8833/MX1508: AIN1/AIN2/BIN1/BIN2 = IN1..IN4 and PWMA/PWMB unused)
constexpr uint8_t kPwmA = 23;  // left motor
constexpr uint8_t kAin1 = 19;
constexpr uint8_t kAin2 = 18;
constexpr uint8_t kPwmB = 14;  // right motor
constexpr uint8_t kBin1 = 5;  // L298N IN3
constexpr uint8_t kBin2 = 4;  // L298N IN4
constexpr uint8_t kStby = 13;  // TB6612 STBY (10k pull-down); unused for L298N / DRV8833

// MPU6050 (I2C)
constexpr uint8_t kSda = 21;
constexpr uint8_t kScl = 22;

// optional slot-disc speed sensors (LM393 modules, digital out)
constexpr uint8_t kEncL = 16;
constexpr uint8_t kEncR = 17;

// operator interface
constexpr uint8_t kButton = 0;      // BOOT button (+ optional external button in parallel), active LOW
constexpr uint8_t kLedRed = 15;     // end-zone LED to GND (330R in series recommended), active HIGH
constexpr uint8_t kLedStatus = 2;   // on-board LED on most DevKit clones (else add LED + 1k), active HIGH
}  // namespace pins

// Battery voltage sensing needs a resistor divider on GPIO26:
// VBAT -47k- GPIO26 -10k- GND (16.8 V -> 2.95 V). Build with -DMESH_VBAT_SENSE=1
// once it is fitted. Without it (default) the core assumes vbat_nominal_v and
// the low-battery check is skipped - measure the pack with a multimeter.
#ifndef MESH_VBAT_SENSE
#define MESH_VBAT_SENSE 0
#endif
constexpr float kVbatDividerRatio = 5.7f;

// LEDC PWM: 20 kHz (inaudible) at 10-bit resolution. L298N switches slowly:
// 1 kHz is kinder to it (set MESH_PWM_HZ=1000 for L298N if motors get hot).
#ifndef MESH_PWM_HZ
#define MESH_PWM_HZ 20000
#endif
constexpr uint32_t kPwmFreqHz = MESH_PWM_HZ;
constexpr uint8_t kPwmBits = 10;
constexpr uint8_t kPwmChA = 0;
constexpr uint8_t kPwmChB = 1;
constexpr uint8_t kPwmChA2 = 2;  // DRV8833 only
constexpr uint8_t kPwmChB2 = 3;
#endif
