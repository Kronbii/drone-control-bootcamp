// s01_rc_plant -- Lab 1, part B: control a real (electrical) plant with easyPID.
// Drone-control bootcamp, session 1 (sessions/01-feedback-and-pid/lab.md). MIT licence.
//
// The plant is an R-C low-pass filter: PWM pin 5 -> 10 kOhm -> node A0 -> 100 uF -> GND.
// Its time constant is RC = 1 s and its static gain is 1 V/V, so it behaves exactly like
// the first-order plant of the notebook:  tau y' + y = K u.
//
//          D5 ----[ 10k ]----+---- A0          (optional 2nd/3rd stage:
//                            |                   A0 --[10k]--+-- A1 --[10k]--+-- A2
//                          [100uF]  (+ to A0)                |               |
//                            |                            [100uF]         [100uF]
//         GND ---------------+                               GND             GND
//
// Units are volts on both sides (the PWM duty cycle is converted to its average
// voltage), so the controller gains are dimensionless and match the notebook.
//
// Commands (115200 baud, newline):
//   pid | open          closed loop (default) or open loop
//   u 2.5               open-loop output (V)
//   sp 2.5              setpoint (V)
//   kp 2  ki 1  kd 0    gains
//   steps 1.5 3.5 8     square-wave setpoint between 1.5 V and 3.5 V, 8 s each half
//   steps off           constant setpoint again
//   tap 0|1|2           measure on A0, A1 or A2 (number of RC stages - 1)
//   ?                   print the settings
// Output: t_ms,sp,y,u,p,i,d   (volts)
#include <easyPID.h>

#include "serial_cmd.h"

const uint8_t PIN_PWM = 5;  // 980 Hz PWM on Uno/Nano: ripple on 100 uF is < 1 mV
const uint8_t TAPS[3] = {A0, A1, A2};
const unsigned long PERIOD_MS = 10;  // 100 Hz
const float VREF = 5.0f;

float kp = 1.0f, ki = 0.0f, kd = 0.0f;
float setpoint = 2.5f, uOpen = 0.0f;
bool closedLoop = true;
uint8_t tap = 0;
float stepLo = 1.5f, stepHi = 3.5f, stepHalf = 8.0f;
bool stepping = false;

PIDController pid(kp, ki, kd, 0.0f, VREF);
SerialCommands cmd;
unsigned long lastMs = 0, stepStart = 0;

float readVolts() {
  long sum = 0;
  for (uint8_t i = 0; i < 4; i++) sum += analogRead(TAPS[tap]);  // average out ADC noise
  return (sum / 4.0f) * VREF / 1023.0f;
}

void writeVolts(float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > VREF) v = VREF;
  analogWrite(PIN_PWM, (int)(v / VREF * 255.0f + 0.5f));
}

void printSettings() {
  Serial.print(closedLoop ? F("# mode=pid") : F("# mode=open"));
  Serial.print(F(" kp="));
  Serial.print(kp, 3);
  Serial.print(F(" ki="));
  Serial.print(ki, 3);
  Serial.print(F(" kd="));
  Serial.print(kd, 3);
  Serial.print(F(" sp="));
  Serial.print(setpoint, 2);
  Serial.print(F(" tap=A"));
  Serial.print(tap);
  Serial.print(stepping ? F(" steps=on") : F(" steps=off"));
  Serial.println();
}

void handleCommand() {
  if (cmd.is("pid")) {
    closedLoop = true;
    pid.reset();
  } else if (cmd.is("open")) {
    closedLoop = false;
  } else if (cmd.is("u")) {
    uOpen = cmd.f(0, uOpen);
  } else if (cmd.is("sp")) {
    setpoint = cmd.f(0, setpoint);
  } else if (cmd.is("kp")) {
    kp = cmd.f(0, kp);
  } else if (cmd.is("ki")) {
    ki = cmd.f(0, ki);
  } else if (cmd.is("kd")) {
    kd = cmd.f(0, kd);
  } else if (cmd.is("steps")) {
    if (strcmp(cmd.s(0), "off") == 0) {
      stepping = false;
    } else if (cmd.count() >= 3) {
      stepLo = cmd.f(0);
      stepHi = cmd.f(1);
      stepHalf = cmd.f(2);
      stepping = true;
      stepStart = millis();
    }
  } else if (cmd.is("tap")) {
    int t = (int)cmd.f(0);
    if (t >= 0 && t <= 2) tap = (uint8_t)t;
  } else if (!cmd.is("?")) {
    Serial.println(F("# unknown command"));
  }
  pid.setTunings(kp, ki, kd);
  printSettings();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWM, OUTPUT);
  writeVolts(0.0f);
  Serial.println(F("# s01_rc_plant: easyPID on an RC low-pass filter"));
  pid.begin();
  printSettings();
  Serial.println(F("t_ms,sp,y,u,p,i,d"));
  lastMs = millis();
}

void loop() {
  if (cmd.poll()) handleCommand();
  unsigned long now = millis();
  if (now - lastMs < PERIOD_MS) return;
  float dtMs = (float)(now - lastMs);
  lastMs = now;

  if (stepping) {
    unsigned long halfMs = (unsigned long)(stepHalf * 1000.0f);
    setpoint = (((now - stepStart) / halfMs) % 2 == 0) ? stepLo : stepHi;
  }
  float y = readVolts();
  float u = closedLoop ? pid.update(setpoint, y, dtMs) : uOpen;
  writeVolts(u);

  Serial.print(now);
  Serial.print(',');
  Serial.print(setpoint, 3);
  Serial.print(',');
  Serial.print(y, 3);
  Serial.print(',');
  Serial.print(u, 3);
  Serial.print(',');
  Serial.print(closedLoop ? pid.getPterm() : 0.0f, 3);
  Serial.print(',');
  Serial.print(closedLoop ? pid.getIterm() : 0.0f, 3);
  Serial.print(',');
  Serial.println(closedLoop ? pid.getDterm() : 0.0f, 3);
}
