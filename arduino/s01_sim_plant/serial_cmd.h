// serial_cmd.h -- tiny non-blocking line parser for lab commands such as "kp 2.5".
// Part of the drone-control bootcamp (MIT). The master copy lives in arduino/common/;
// tools/sync_sketch_headers.py copies it next to every sketch that uses it.
#pragma once

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

class SerialCommands {
 public:
  // Call every loop. Returns true once a complete, non-empty line has arrived;
  // then use is(), count(), f() and s() before the next call.
  bool poll() {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        buf_[len_] = '\0';
        len_ = 0;
        split();
        return argc_ > 0;
      }
      if (len_ < sizeof(buf_) - 1) buf_[len_++] = c;
    }
    return false;
  }
  bool is(const char* name) const { return argc_ > 0 && strcmp(argv_[0], name) == 0; }
  uint8_t count() const { return argc_ > 0 ? argc_ - 1 : 0; }  // number of arguments
  float f(uint8_t i, float fallback = 0.0f) const {
    return (i + 1 < argc_) ? (float)atof(argv_[i + 1]) : fallback;
  }
  const char* s(uint8_t i) const { return (i + 1 < argc_) ? argv_[i + 1] : ""; }
  const char* name() const { return argc_ > 0 ? argv_[0] : ""; }

 private:
  void split() {
    argc_ = 0;
    char* p = strtok(buf_, " \t,");
    while (p != NULL && argc_ < kMaxArgs) {
      argv_[argc_++] = p;
      p = strtok(NULL, " \t,");
    }
  }
  static const uint8_t kMaxArgs = 5;
  char buf_[48] = {0};
  uint8_t len_ = 0;
  char* argv_[kMaxArgs] = {0};
  uint8_t argc_ = 0;
};
