// s03_rig_angle_pid -- Lab 3: hold the seesaw at an angle with one easyPID loop.
// Drone-control bootcamp, session 3 (sessions/03-the-1dof-rig/lab.md). MIT licence.
//
// READ hardware/rig/safety.md FIRST. Props on only after the props-off checks of
// s03_esc_bench, guards fitted, rig clamped, e-stop within reach, nobody in the prop plane.
//
// Loop (200 Hz): pot angle -> easyPID -> differential command u (us) -> mixer
//   motor 1 = base + u, motor 2 = base - u   (base ramps up over 2 s after arming)
// The gains are in microseconds of ESC pulse per degree: the same units as the
// notebook model (bootcamp.rig), so a simulated tuning transfers directly.
//
// Commands (115200 baud):
//   kp 0.5 ki 0 kd 0.3     gains                   sp 10       setpoint (deg)
//   filter 0.6 | off       derivative EMA filter    base 1350   base throttle (us)
//   seq                    step test: 0, +10, -10, 0 deg, 4 s each (for the logger)
//   kick                   disturbance: +40 us on u for 100 ms (same as the capstone test)
//   fault 0.7 | fault off  scale motor 2's throttle above idle (fault-injection, lab 6)
//   zero                   record the current pot reading as level
//   x                      disarm now          ?   settings
// Output (50 Hz): t_ms,sp,angle,u,us1,us2,p,i,d,armed
#include <easyPID.h>

#include "esc_timer1.h"
#include "rig_safety.h"
#include "serial_cmd.h"

// ---- calibration: set from the "zero" command and the sign check in the lab
int potCenter = 512;
float potSign = 1.0f;

// ---- control settings
const unsigned long LOOP_US = 5000;  // 200 Hz
const float U_LIMIT = 200.0f;        // max differential (us)
const float US_LO = US_IDLE, US_HI = US_CAP;
// Conservative start (lab step 1). Notebook 3 shows why P alone is not enough here.
float kp = 0.5f, ki = 0.0f, kd = 0.3f;
float baseUs = 1350.0f;
float setpoint = 0.0f;
float faultEta = 1.0f;  // 1 = healthy

PIDController pid(kp, ki, kd, -U_LIMIT, U_LIMIT);
RigSafety safety;
SerialCommands cmd;

bool seqActive = false;
unsigned long seqStart = 0;
unsigned long lastLoop = 0;
uint8_t logDivider = 0;
unsigned long kickUntil = 0;  // "kick": +40 us on u for 100 ms, a repeatable disturbance
const float KICK_US = 40.0f;
bool wasRamping = true;

float potAngle(int counts) { return (counts - potCenter) * POT_DEG_PER_COUNT * potSign; }

void printSettings() {
  Serial.print(F("# kp="));
  Serial.print(kp, 3);
  Serial.print(F(" ki="));
  Serial.print(ki, 3);
  Serial.print(F(" kd="));
  Serial.print(kd, 3);
  Serial.print(F(" sp="));
  Serial.print(setpoint, 1);
  Serial.print(F(" base="));
  Serial.print(baseUs, 0);
  Serial.print(F(" fault_eta="));
  Serial.print(faultEta, 2);
  Serial.print(F(" pot_center="));
  Serial.println(potCenter);
}

void handleCommand(bool& stop) {
  if (cmd.is("x") || cmd.is("stop")) stop = true;
  else if (cmd.is("kp")) kp = cmd.f(0, kp);
  else if (cmd.is("ki")) ki = cmd.f(0, ki);
  else if (cmd.is("kd")) kd = cmd.f(0, kd);
  else if (cmd.is("sp")) {
    setpoint = constrain(cmd.f(0, setpoint), -20.0f, 20.0f);
    seqActive = false;
  } else if (cmd.is("base")) baseUs = constrain(cmd.f(0, baseUs), (float)US_IDLE, 1500.0f);
  else if (cmd.is("filter")) {
    if (strcmp(cmd.s(0), "off") == 0) pid.setDerivativeFilter(FILTER_NONE);
    else pid.setDerivativeFilter(FILTER_EMA, cmd.f(0, 0.6f));
  } else if (cmd.is("seq")) {
    seqActive = true;
    seqStart = millis();
  } else if (cmd.is("kick")) {
    kickUntil = millis() + 100;
  } else if (cmd.is("fault")) {
    faultEta = (strcmp(cmd.s(0), "off") == 0) ? 1.0f : constrain(cmd.f(0, 1.0f), 0.3f, 1.0f);
  } else if (cmd.is("zero")) {
    potCenter = analogRead(PIN_POT);
  } else if (!cmd.is("?")) {
    Serial.println(F("# unknown command"));
  }
  pid.setTunings(kp, ki, kd);
  printSettings();
}

float sequenceSetpoint(unsigned long now) {
  const float steps[4] = {0.0f, 10.0f, -10.0f, 0.0f};
  unsigned long k = (now - seqStart) / 4000UL;
  if (k >= 4) {
    seqActive = false;
    Serial.println(F("# SEQ DONE"));
    return 0.0f;
  }
  return steps[k];
}

void setup() {
  Serial.begin(115200);
  safety.begin();
  pid.setAntiWindup(ANTIWINDUP_CLAMP);
  pid.setDerivativeFilter(FILTER_EMA, 0.6f);
  pid.begin();
  Serial.println(F("# s03_rig_angle_pid: single-loop angle control. Arm: switch OFF -> ON."));
  printSettings();
  Serial.println(F("t_ms,sp,angle,u,us1,us2,p,i,d,armed"));
  lastLoop = micros();
}

void loop() {
  bool stop = false;
  if (cmd.poll()) handleCommand(stop);

  unsigned long nowUs = micros();
  if (nowUs - lastLoop < LOOP_US) return;
  float dtMs = (nowUs - lastLoop) / 1000.0f;
  lastLoop = nowUs;
  unsigned long now = millis();

  int counts = analogRead(PIN_POT);
  float angle = potAngle(counts);
  bool armed = safety.update(counts, angle, stop);
  if (seqActive) setpoint = sequenceSetpoint(now);

  float u = 0.0f, m1 = ESC_US_MIN, m2 = ESC_US_MIN;
  if (armed) {
    float r = safety.ramp();
    if (r < 1.0f) {
      // spin-up: no feedback yet, both motors follow the ramp
      m1 = m2 = US_IDLE + r * (baseUs - US_IDLE);
      wasRamping = true;
    } else {
      if (wasRamping) {
        pid.reset();  // start the loop clean once the motors are up to speed
        wasRamping = false;
      }
      u = pid.update(setpoint, angle, dtMs);
      if ((long)(kickUntil - now) > 0) u += KICK_US;
      mixDifferential(baseUs, u, US_LO, US_HI, m1, m2);
      // fault injection: motor 2 delivers only a fraction of its throttle above idle
      m2 = US_IDLE + faultEta * (m2 - US_IDLE);
    }
  } else {
    wasRamping = true;
  }
  safety.writeMotors(m1, m2);

  if (++logDivider >= 4) {  // 50 Hz log
    logDivider = 0;
    Serial.print(now);
    Serial.print(',');
    Serial.print(setpoint, 1);
    Serial.print(',');
    Serial.print(angle, 2);
    Serial.print(',');
    Serial.print(u, 1);
    Serial.print(',');
    Serial.print(armed ? m1 : ESC_US_MIN, 0);
    Serial.print(',');
    Serial.print(armed ? m2 : ESC_US_MIN, 0);
    Serial.print(',');
    Serial.print(pid.getPterm(), 1);
    Serial.print(',');
    Serial.print(pid.getIterm(), 1);
    Serial.print(',');
    Serial.print(pid.getDterm(), 1);
    Serial.print(',');
    Serial.println(armed ? 1 : 0);
  }
}
