// rig_safety.h -- pins, throttle limits and the arming state machine of the seesaw rig.
// Part of the drone-control bootcamp (MIT). Master copy in arduino/common/.
//
// The software interlocks here are the SECOND line of defence. The first is physical:
// the emergency-stop switch in the 12 V line to the ESCs, prop guards, the rig clamped
// to the bench, and nobody inside the prop plane. See hardware/rig/safety.md.
//
// The arming rules are the same as notebook exercise 3.6:
//   arm only on an OFF->ON transition of the arm switch, with a plausible pot reading,
//   the beam within +-20 deg of level and at least 3 s after boot (ESCs initialised);
//   disarm when the switch goes off, the pot reading becomes implausible (wire off),
//   the beam stays beyond +-28 deg (near the stops) for 300 ms, or "x" is sent.
#pragma once

#include <Arduino.h>

#include "esc_timer1.h"

// ---------------------------------------------------------------- pins
const uint8_t PIN_POT = A0;  // pivot potentiometer wiper
const uint8_t PIN_ARM = 2;   // arm switch to GND, INPUT_PULLUP: closed = ON
const uint8_t PIN_LED = 13;  // on while armed
// ESC 1 (motor 1, the "+" end) on pin 9, ESC 2 on pin 10 -- see esc_timer1.h

// ---------------------------------------------------------------- limits
const uint16_t US_IDLE = 1100;  // armed minimum: props turn slowly
const uint16_t US_CAP = 1600;   // software throttle cap, never exceeded
const unsigned long RAMP_MS = 2000;  // base throttle ramps up over this time after arming

// ---------------------------------------------------------------- arming rules
const int POT_MIN_VALID = 10;
const int POT_MAX_VALID = 1013;
const float ARM_MAX_ANGLE_DEG = 20.0f;
const unsigned long ARM_MIN_BOOT_MS = 3000;
const float TRIP_ANGLE_DEG = 28.0f;
const unsigned long TRIP_TIME_MS = 300;

// Pot calibration: 270-degree pot on a 10-bit ADC. Set POT_CENTER with the "zero"
// command (beam level) and POT_SIGN so that lifting the motor-1 end gives +angle.
const float POT_DEG_PER_COUNT = 270.0f / 1023.0f;

inline bool potPlausible(int counts) { return counts >= POT_MIN_VALID && counts <= POT_MAX_VALID; }

inline bool armingAllowed(bool switchOn, bool switchWasOff, int potCounts, float angleDeg,
                          unsigned long msSinceBoot) {
  return switchOn && switchWasOff && potPlausible(potCounts) &&
         fabs(angleDeg) <= ARM_MAX_ANGLE_DEG && msSinceBoot >= ARM_MIN_BOOT_MS;
}

class RigSafety {
 public:
  void begin(uint16_t escFrameHz = 250) {
    pinMode(PIN_ARM, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    escBegin(escFrameHz);  // ESCs see 1000 us (stop) from now on
  }

  bool switchOn() const { return digitalRead(PIN_ARM) == LOW; }

  // Call once per control loop. Returns true while armed.
  bool update(int potCounts, float angleDeg, bool stopRequested) {
    unsigned long now = millis();
    bool sw = switchOn();
    if (!sw) switchWasOff_ = true;
    if (armed_) {
      const char* why = NULL;
      if (!sw) {
        why = "arm switch off";
      } else if (!potPlausible(potCounts)) {
        why = "pot reading implausible (wire loose?)";
      } else if (fabs(angleDeg) > TRIP_ANGLE_DEG) {
        if (tripSince_ == 0) tripSince_ = now;
        if (now - tripSince_ >= TRIP_TIME_MS) why = "beam at the stops";
      } else {
        tripSince_ = 0;
      }
      if (stopRequested) why = "stop command";
      if (why != NULL) {
        armed_ = false;
        switchWasOff_ = false;  // the switch must go OFF before the next arming
        escStopAll();
        Serial.print(F("# DISARMED: "));
        Serial.println(why);
      }
    } else if (armingAllowed(sw, switchWasOff_, potCounts, angleDeg, now)) {
      armed_ = true;
      armedAt_ = now;
      tripSince_ = 0;
      Serial.println(F("# ARMED"));
    }
    digitalWrite(PIN_LED, armed_ ? HIGH : LOW);
    return armed_;
  }

  bool armed() const { return armed_; }

  // 0 -> 1 over RAMP_MS after arming; use it to bring the base throttle up gently.
  float ramp() const {
    if (!armed_) return 0.0f;
    unsigned long dt = millis() - armedAt_;
    return dt >= RAMP_MS ? 1.0f : (float)dt / (float)RAMP_MS;
  }

  // The only function that writes to the ESCs while the rig runs.
  void writeMotors(float us1, float us2) {
    if (!armed_) {
      escStopAll();
      return;
    }
    escWriteUs(0, clampUs(us1));
    escWriteUs(1, clampUs(us2));
  }

  static uint16_t clampUs(float us) {
    if (us < US_IDLE) us = US_IDLE;
    if (us > US_CAP) us = US_CAP;
    return (uint16_t)(us + 0.5f);
  }

 private:
  bool armed_ = false;
  bool switchWasOff_ = false;
  unsigned long armedAt_ = 0;
  unsigned long tripSince_ = 0;
};

// Shift the base so that base +- u fits in [lo, hi] (keeps the torque, like
// Betaflight's air mode); only if 2|u| exceeds the span is the differential reduced.
// Same as bootcamp.rig.mix and notebook exercise 3.4.
inline void mixDifferential(float base, float u, float lo, float hi, float& m1, float& m2) {
  float half = 0.5f * (hi - lo);
  if (u > half) u = half;
  if (u < -half) u = -half;
  m1 = base + u;
  m2 = base - u;
  float top = m1 > m2 ? m1 : m2;
  float bot = m1 > m2 ? m2 : m1;
  float shift = 0.0f;
  if (top > hi) {
    shift = hi - top;
  } else if (bot < lo) {
    shift = lo - bot;
  }
  m1 += shift;
  m2 += shift;
}
