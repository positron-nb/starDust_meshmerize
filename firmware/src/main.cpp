// main.cpp - ESP32 entry point. All behaviour lives in mesh::Robot
// (src/app/robot.*), which is the same code the simulator runs.
//
// Competition compliance: the Wi-Fi and Bluetooth radios are never started
// (no WiFi.h / BLE code is linked); btStop() is called defensively at boot.
// The rules require Wi-Fi/Bluetooth devices near the arena to be OFF.
//
// Loop timing: a fixed 1 kHz step driven by micros() on core 1. Nine
// analogRead() calls take roughly 0.1-0.2 ms on a 240 MHz ESP32 - measure it
// with the debug build (firmware/README.md).
#ifdef ARDUINO
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <string.h>

#include "app/robot.h"
#include "hal_esp32/hal_esp32.h"
#include "util/util.h"

namespace {
mesh::Robot g_robot;
mesh::HalEsp32 g_hal;
uint8_t g_blob[mesh::kPersistMaxBytes];
uint32_t g_next_us = 0;
uint32_t g_overruns = 0;

#ifdef MESH_DEBUG_SERIAL
void serialSink(const char* line) { Serial.println(line); }

// 20 Hz telemetry for bench calibration (docs/calibration_guide.md):
// TEL <ms> s=<mm> th=<deg> gyro=<dps> encL encR pos=<mm> on=<n> v=array..|wingL,wingR raw=... vbat trim state
void telemetry(const mesh::HalInputs& in) {
  static uint32_t last = 0;
  uint32_t ms = millis();
  if (ms - last < 50) return;
  last = ms;
  const mesh::LineFeatures& f = g_robot.features();
  const mesh::Odometry& o = g_robot.odometry();
  Serial.printf("TEL %lu s=%.1f th=%.1f gyro=%.1f%s enc=%ld,%ld pos=%.1f on=%u v=%u,%u,%u,%u,%u|%u,%u "
                "raw=%u,%u,%u,%u,%u|%u,%u vbat=%.2f trim=%+.3f st=%d\n",
                (unsigned long)ms, o.s(), o.theta() * 57.2958f, in.gyro_z_dps, in.gyro_ok ? "" : "(none)",
                (long)in.enc_left, (long)in.enc_right, f.pos_mm, f.n_on, f.val[0], f.val[1], f.val[2], f.val[3],
                f.val[4], f.val[5], f.val[6], in.raw[0], in.raw[1], in.raw[2], in.raw[3], in.raw[4], in.raw[5],
                in.raw[6], in.vbat, g_robot.motorTrim(), (int)g_robot.state());
}
#endif

void watchdogInit() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t c = {};
  c.timeout_ms = 1000;
  c.idle_core_mask = 0;
  c.trigger_panic = true;
  esp_task_wdt_reconfigure(&c);
#else
  esp_task_wdt_init(1, true);  // 1 s, reset on timeout
#endif
  esp_task_wdt_add(NULL);  // watch the loop task
}
}  // namespace

void setup() {
  g_hal.begin();  // motors disabled first
  btStop();       // radios stay off (they are never started anyway)
#ifdef MESH_DEBUG_SERIAL
  Serial.begin(115200);
  mesh::Log::setSink(serialSink);
#endif
  g_robot.init();
  size_t n = g_hal.loadStorage(g_blob, sizeof(g_blob));
  g_robot.loadPersisted(g_blob, n);
  watchdogInit();
  g_next_us = micros();
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - g_next_us) < 0) return;
  g_next_us += mesh::kControlPeriodUs;
  if ((int32_t)(now - g_next_us) > (int32_t)(5 * mesh::kControlPeriodUs)) {
    g_next_us = now + mesh::kControlPeriodUs;  // fell far behind: resynchronise
    ++g_overruns;
  }
  mesh::HalInputs in;
  mesh::HalOutputs out;
  memset(&out, 0, sizeof(out));
  g_hal.read(in);
  g_robot.step(in, out);
  g_hal.write(out);
#ifdef MESH_DEBUG_SERIAL
  telemetry(in);
#endif
  if (out.save_request) {
    // Robot is standing still; an NVS write takes a few to tens of ms.
    g_hal.stopMotors();
    size_t n = g_robot.serialize(g_blob, sizeof(g_blob));
    bool ok = n > 0 && g_hal.saveStorage(g_blob, n);
    mesh::Log::printf("STORE %s (%u bytes)", ok ? "saved" : "FAILED", (unsigned)n);
    g_next_us = micros();
  }
  esp_task_wdt_reset();
}
#endif  // ARDUINO
