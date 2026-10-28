#include "util.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace mesh {

uint32_t crc32(const uint8_t* data, size_t len, uint32_t crc) {
  crc = ~crc;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

char Log::buf_[Log::kLines][Log::kLen];
int Log::head_ = 0;
int Log::count_ = 0;
uint32_t Log::time_ms_ = 0;
Log::Sink Log::sink_ = nullptr;

void Log::printf(const char* fmt, ...) {
  int idx = (head_ + count_) % kLines;
  if (count_ == kLines) {  // overwrite oldest
    head_ = (head_ + 1) % kLines;
  } else {
    ++count_;
  }
  int n = snprintf(buf_[idx], kLen, "%7.3f ", time_ms_ / 1000.0);
  if (n < 0) n = 0;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf_[idx] + n, kLen - n, fmt, ap);
  va_end(ap);
  if (sink_) sink_(buf_[idx]);
}

bool Log::pop(char* out, int cap) {
  if (count_ == 0) return false;
  strncpy(out, buf_[head_], cap - 1);
  out[cap - 1] = 0;
  head_ = (head_ + 1) % kLines;
  --count_;
  return true;
}

void Log::clear() { head_ = count_ = 0; }

}  // namespace mesh
