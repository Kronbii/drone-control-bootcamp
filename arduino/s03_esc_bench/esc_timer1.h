// esc_timer1.h -- two ESC outputs on pins 9 and 10 using Timer1 (ATmega328P: Uno, Nano).
// Part of the drone-control bootcamp (MIT). Master copy in arduino/common/.
//
// Standard "servo PWM" ESC protocol: a 1000-2000 us pulse every frame. Timer1 runs in
// fast-PWM mode 14 (TOP = ICR1) with a /8 prescaler, so at 16 MHz one tick is 0.5 us
// and the pulse width is exact and jitter-free, unlike software-timed servo pulses.
// The frame rate (default 250 Hz) is far above the 50 Hz of the Servo library, which
// cuts the actuator delay; nearly all multirotor ESCs accept it. If yours misbehaves,
// use escBegin(50).
//
// Timer1 is also used by the Servo library and by analogWrite() on pins 9/10: do not
// use those together with this file.
#pragma once

#include <Arduino.h>

#if !defined(__AVR_ATmega328P__)
#error "esc_timer1.h drives Timer1 of the ATmega328P (Arduino Uno / Nano)."
#endif

const uint16_t ESC_US_MIN = 1000;
const uint16_t ESC_US_MAX = 2000;
static const uint16_t kEscTicksPerUs = (uint16_t)(F_CPU / 8000000UL);

inline void escWriteUs(uint8_t channel, uint16_t us) {
  if (us < ESC_US_MIN) us = ESC_US_MIN;
  if (us > ESC_US_MAX) us = ESC_US_MAX;
  uint16_t ticks = us * kEscTicksPerUs;
  if (channel == 0) {
    OCR1A = ticks;  // pin 9
  } else {
    OCR1B = ticks;  // pin 10
  }
}

inline void escBegin(uint16_t frameHz = 250) {
  if (frameHz < 50) frameHz = 50;
  if (frameHz > 400) frameHz = 400;  // a 2000 us pulse must fit in the frame
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  ICR1 = (uint16_t)((F_CPU / 8UL) / frameHz - 1);
  escWriteUs(0, ESC_US_MIN);
  escWriteUs(1, ESC_US_MIN);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
}

inline void escStopAll() {
  escWriteUs(0, ESC_US_MIN);
  escWriteUs(1, ESC_US_MIN);
}
