// s02_rc_tuning -- Lab 2: real-world PID on a 3-stage RC ladder.
// Drone-control bootcamp, session 2 (sessions/02-tuning-and-real-world-pid/lab.md). MIT.
//
// Plant: three 10 kOhm / 100 uF stages in cascade (see s01_rc_plant for the drawing);
// measure on A2. Its transfer function is 1 / (1 + 6 sRC + 5 (sRC)^2 + (sRC)^3) with
// RC = 1 s: ultimate gain 29 at 2.45 rad/s (period 2.57 s). Lab 2 measures this three ways.
//
// Commands (115200 baud, newline):
//   pid | open | u 2.5            closed loop, open loop, open-loop output (V)
//   sp 2.5 | kp 2 | ki 1 | kd 0    setpoint and gains
//   steptest 1 3 20               open-loop step 1 V -> 3 V, logged for 20 s
//   aw none|clamp|backcalc        easyPID anti-windup mode (default clamp)
//   filter 0.7 | filter off       easyPID EMA derivative filter
//   lim 0 5                       output limits (V); e.g. "lim 0 3" to force saturation
//   ramp 0.05 | ramp off          setpoint ramp: max change per sample (V)
//   ts 10                         sample period (ms)
//   tune 2.0 0.01                 relay autotune: relay amplitude 2 V, noise band 0.01 V
//   rule zn|tl|pe|no              apply a tuning rule from the last autotune
//   tap 0|1|2  ?                  measurement node, print settings
// Output: t_ms,sp,y,u,p,i,d,mode    (mode 0 pid, 1 open, 2 relay)
#include <PIDTuner.h>
#include <easyPID.h>

#include "serial_cmd.h"

const uint8_t PIN_PWM = 5;
const uint8_t TAPS[3] = {A0, A1, A2};
const float VREF = 5.0f;

enum Mode { MODE_PID = 0, MODE_OPEN = 1, MODE_RELAY = 2 };

float kp = 2.0f, ki = 0.5f, kd = 0.0f;
float setpoint = 2.5f, spRamped = 2.5f, rampStep = 0.0f;
float uOpen = 0.0f, outLo = 0.0f, outHi = VREF;
unsigned long periodMs = 10;
uint8_t tap = 2;
Mode mode = MODE_PID;

// step test
bool stepTest = false;
float stepU1 = 0.0f;
unsigned long stepAt = 0, stepEnd = 0;

// two-pass relay autotune (pass 1 lets the limit cycle settle, pass 2 measures it)
uint8_t tunePass = 0;
float relayAmp = 2.0f, noiseBand = 0.01f;

PIDController pid(kp, ki, kd, outLo, outHi);
PIDTuner tuner(pid);
SerialCommands cmd;
unsigned long lastMs = 0;

float readVolts() {
  long sum = 0;
  for (uint8_t i = 0; i < 4; i++) sum += analogRead(TAPS[tap]);
  return (sum / 4.0f) * VREF / 1023.0f;
}

void writeVolts(float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > VREF) v = VREF;
  analogWrite(PIN_PWM, (int)(v / VREF * 255.0f + 0.5f));
}

void printTuning() {
  Serial.print(F("# autotune: Ku="));
  Serial.print(tuner.getUltimateGain(), 3);
  Serial.print(F(" Pu="));
  Serial.print(tuner.getUltimatePeriod(), 3);
  Serial.println(F(" s"));
  const TuningRule rules[4] = {TUNING_ZIEGLER_NICHOLS, TUNING_TYREUS_LUYBEN, TUNING_PESSEN,
                               TUNING_NO_OVERSHOOT};
  const char* names[4] = {"zn", "tl", "pe", "no"};
  for (uint8_t r = 0; r < 4; r++) {
    float p, i, d;
    if (tuner.getTunings(p, i, d, rules[r])) {
      Serial.print(F("#   "));
      Serial.print(names[r]);
      Serial.print(F(": kp="));
      Serial.print(p, 3);
      Serial.print(F(" ki="));
      Serial.print(i, 3);
      Serial.print(F(" kd="));
      Serial.println(d, 3);
    }
  }
}

void printSettings() {
  Serial.print(F("# mode="));
  Serial.print((int)mode);
  Serial.print(F(" kp="));
  Serial.print(kp, 3);
  Serial.print(F(" ki="));
  Serial.print(ki, 3);
  Serial.print(F(" kd="));
  Serial.print(kd, 3);
  Serial.print(F(" sp="));
  Serial.print(setpoint, 2);
  Serial.print(F(" lim="));
  Serial.print(outLo, 2);
  Serial.print('/');
  Serial.print(outHi, 2);
  Serial.print(F(" ts="));
  Serial.print(periodMs);
  Serial.print(F(" ramp="));
  Serial.print(rampStep, 3);
  Serial.print(F(" tap=A"));
  Serial.println(tap);
}

void applyRule(const char* name) {
  TuningRule r = TUNING_TYREUS_LUYBEN;
  if (strcmp(name, "zn") == 0) r = TUNING_ZIEGLER_NICHOLS;
  else if (strcmp(name, "pe") == 0) r = TUNING_PESSEN;
  else if (strcmp(name, "no") == 0) r = TUNING_NO_OVERSHOOT;
  float p, i, d;
  if (tuner.getTunings(p, i, d, r)) {
    kp = p;
    ki = i;
    kd = d;
    pid.setTunings(kp, ki, kd);
    pid.reset();
    mode = MODE_PID;
  } else {
    Serial.println(F("# no autotune result yet: run 'tune' first"));
  }
}

void handleCommand() {
  if (cmd.is("pid")) {
    mode = MODE_PID;
    pid.reset();
  } else if (cmd.is("open")) {
    mode = MODE_OPEN;
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
  } else if (cmd.is("steptest") && cmd.count() >= 3) {
    mode = MODE_OPEN;
    uOpen = cmd.f(0);
    stepU1 = cmd.f(1);
    stepAt = millis() + 2000;  // 2 s of baseline first
    stepEnd = stepAt + (unsigned long)(cmd.f(2) * 1000.0f);
    stepTest = true;
  } else if (cmd.is("aw")) {
    if (strcmp(cmd.s(0), "none") == 0) pid.setAntiWindup(ANTIWINDUP_NONE);
    else if (strcmp(cmd.s(0), "backcalc") == 0) pid.setAntiWindup(ANTIWINDUP_BACKCALC);
    else pid.setAntiWindup(ANTIWINDUP_CLAMP);
    Serial.print(F("# anti-windup "));
    Serial.println(cmd.s(0));
  } else if (cmd.is("filter")) {
    if (strcmp(cmd.s(0), "off") == 0) pid.setDerivativeFilter(FILTER_NONE);
    else pid.setDerivativeFilter(FILTER_EMA, cmd.f(0, 0.7f));
  } else if (cmd.is("lim") && cmd.count() >= 2) {
    outLo = cmd.f(0);
    outHi = cmd.f(1);
    pid.setOutputLimits(outLo, outHi);
  } else if (cmd.is("ramp")) {
    rampStep = (strcmp(cmd.s(0), "off") == 0) ? 0.0f : cmd.f(0, 0.0f);
  } else if (cmd.is("ts") && cmd.f(0) >= 2.0f) {
    periodMs = (unsigned long)cmd.f(0);
  } else if (cmd.is("tune")) {
    relayAmp = cmd.f(0, relayAmp);
    noiseBand = cmd.f(1, noiseBand);
    // outputBias = setpoint: the plant's static gain is 1 V/V, so a drive equal to the
    // setpoint holds it there and the relay swings symmetrically around it.
    if (tuner.start(setpoint, relayAmp, noiseBand, setpoint)) {
      tunePass = 1;
      mode = MODE_RELAY;
      Serial.println(F("# autotune pass 1 (settling the limit cycle)"));
    }
  } else if (cmd.is("rule")) {
    applyRule(cmd.s(0));
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
  Serial.println(F("# s02_rc_tuning: easyPID + PIDTuner on a 3-stage RC ladder"));
  pid.begin();
  printSettings();
  Serial.println(F("t_ms,sp,y,u,p,i,d,mode"));
  lastMs = millis();
}

void loop() {
  if (cmd.poll()) handleCommand();
  unsigned long now = millis();
  if (now - lastMs < periodMs) return;
  float dtMs = (float)(now - lastMs);
  lastMs = now;

  float y = readVolts();
  float u = 0.0f;

  if (rampStep > 0.0f) {  // setpoint ramp, as in easyPID's tuning guide
    float d = setpoint - spRamped;
    if (d > rampStep) d = rampStep;
    if (d < -rampStep) d = -rampStep;
    spRamped += d;
  } else {
    spRamped = setpoint;
  }

  if (mode == MODE_RELAY) {
    u = tuner.update(y);
    TunerState st = tuner.getState();
    if (st == TUNER_COMPLETE || st == TUNER_FAILED) {
      if (tunePass == 1) {
        // Restart at once: the plant is now oscillating, so pass 2 measures the
        // settled limit cycle instead of the growing one (see notebook 2).
        tuner.start(setpoint, relayAmp, noiseBand, setpoint);
        tunePass = 2;
        Serial.println(F("# autotune pass 2 (measuring)"));
        u = setpoint;
      } else {
        tunePass = 0;
        mode = MODE_OPEN;
        uOpen = setpoint;
        if (tuner.isComplete()) printTuning();
        else Serial.println(F("# autotune FAILED: no usable limit cycle (larger amplitude?)"));
      }
    }
  } else if (mode == MODE_OPEN) {
    if (stepTest && (long)(now - stepAt) >= 0) uOpen = stepU1;
    if (stepTest && (long)(now - stepEnd) >= 0) stepTest = false;
    u = uOpen;
  } else {
    u = pid.update(spRamped, y, dtMs);
  }
  writeVolts(u);

  Serial.print(now);
  Serial.print(',');
  Serial.print(spRamped, 3);
  Serial.print(',');
  Serial.print(y, 3);
  Serial.print(',');
  Serial.print(u, 3);
  Serial.print(',');
  Serial.print(pid.getPterm(), 3);
  Serial.print(',');
  Serial.print(pid.getIterm(), 3);
  Serial.print(',');
  Serial.print(pid.getDterm(), 3);
  Serial.print(',');
  Serial.println((int)mode);
}
