// s07_capstone -- Capstone, track R: your controller on the rig, plus the acceptance test.
// Drone-control bootcamp, session 7 (sessions/07-capstone/README.md). MIT licence.
//
// Starts as the cascaded controller of s04_rig_cascaded. Teams change the gains (by
// command or in the constants below) and may fill in the two TEAM HOOKS. Send "test"
// to run the acceptance test; log it with tools/serial_logger.py and score it with
// notebook 7, which runs the *same* test on the rig model (bootcamp/capstone.py).
//
// Acceptance test (seconds after "test"):  hold 0 | 3 s: +15 deg | 8 s: -10 deg |
//   13 s: 0 deg | 20 s: disturbance pulse (+40 us for 100 ms) | 25 s: motor-2 fault
//   (throttle above idle x0.75) | 29 s: +10 deg with the fault | 33 s: end, fault off.
//
// Commands: test | akp aki rkp rki rkd | src pot|imu | tau | sp | base | zero | x | ?
// Output (50 Hz): t_ms,test_t,sp,pot,est,rate_sp,rate,u,us1,us2,armed
#include <easyPID.h>

#include "beam_imu.h"
#include "esc_timer1.h"
#include "mpu6050_lite.h"
#include "rig_safety.h"
#include "serial_cmd.h"

// ---- acceptance test: mirrors bootcamp/capstone.py (tests/test_sketches.py checks it)
const float TEST_T[4] = {0.0f, 3.0f, 8.0f, 13.0f};
const float TEST_SP[4] = {0.0f, 15.0f, -10.0f, 0.0f};
const float TEST_END_S = 33.0f;
const float DIST_TIME_S = 20.0f;
const float DIST_US = 40.0f;
const unsigned long DIST_MS = 100;
const float FAULT_TIME_S = 25.0f;
const float FAULT_ETA = 0.75f;
const float FAULT_STEP_TIME_S = 29.0f;
const float FAULT_STEP_DEG = 10.0f;

// ---- calibration
int potCenter = 512;
float potSign = 1.0f;

// ---- controller (team: start from your notebook-7 tuning)
const unsigned long LOOP_US = 5000;  // 200 Hz
const float U_LIMIT = 200.0f;
const float RATE_LIMIT = 150.0f;
float akp = 4.0f, aki = 0.0f;
float rkp = 0.8f, rki = 0.6f, rkd = 0.01f;
float baseUs = 1350.0f, setpoint = 0.0f;
bool useImu = false;

PIDController anglePid(akp, aki, 0.0f, -RATE_LIMIT, RATE_LIMIT);
PIDController ratePid(rkp, rki, rkd, -U_LIMIT, U_LIMIT);
BeamImu imu;
RigSafety safety;
SerialCommands cmd;

bool testActive = false, wasRamping = true;
unsigned long testStart = 0, lastLoop = 0;
uint8_t logDivider = 0;

// =============================== TEAM HOOKS ===============================
// HOOK 1: feedforward added to the differential command (us), e.g. to cancel a known
// gravity torque: return k * sin(setpoint). Return 0 to disable.
float teamFeedforward(float setpointDeg, float angleDeg) {
  (void)setpointDeg;
  (void)angleDeg;
  return 0.0f;
}

// HOOK 2: called every loop before the controllers, e.g. gain scheduling on the angle.
void teamSchedule(float setpointDeg, float angleDeg) {
  (void)setpointDeg;
  (void)angleDeg;
}
// ==========================================================================

float potAngle(int counts) { return (counts - potCenter) * POT_DEG_PER_COUNT * potSign; }

float testSetpoint(float ts) {
  float sp = 0.0f;
  for (uint8_t k = 0; k < 4; k++) {
    if (ts >= TEST_T[k]) sp = TEST_SP[k];
  }
  if (ts >= FAULT_STEP_TIME_S) sp = FAULT_STEP_DEG;
  return sp;
}

void printSettings() {
  Serial.print(F("# angle kp="));
  Serial.print(akp, 3);
  Serial.print(F(" ki="));
  Serial.print(aki, 3);
  Serial.print(F(" | rate kp="));
  Serial.print(rkp, 3);
  Serial.print(F(" ki="));
  Serial.print(rki, 3);
  Serial.print(F(" kd="));
  Serial.print(rkd, 4);
  Serial.println(useImu ? F(" | src=imu") : F(" | src=pot"));
}

void handleCommand(bool& stop) {
  if (cmd.is("x") || cmd.is("stop")) {
    stop = true;
    testActive = false;
  } else if (cmd.is("test")) {
    if (safety.armed() && safety.ramp() >= 1.0f) {
      testActive = true;
      testStart = millis();
      Serial.println(F("# TEST START"));
    } else {
      Serial.println(F("# arm first and wait for the spin-up"));
    }
  } else if (cmd.is("akp")) akp = cmd.f(0, akp);
  else if (cmd.is("aki")) aki = cmd.f(0, aki);
  else if (cmd.is("rkp")) rkp = cmd.f(0, rkp);
  else if (cmd.is("rki")) rki = cmd.f(0, rki);
  else if (cmd.is("rkd")) rkd = cmd.f(0, rkd);
  else if (cmd.is("src")) useImu = strcmp(cmd.s(0), "imu") == 0;
  else if (cmd.is("tau")) imu.setTimeConstant(cmd.f(0, 0.5f));
  else if (cmd.is("sp")) setpoint = constrain(cmd.f(0, setpoint), -20.0f, 20.0f);
  else if (cmd.is("base")) baseUs = constrain(cmd.f(0, baseUs), (float)US_IDLE, 1500.0f);
  else if (cmd.is("zero")) potCenter = analogRead(PIN_POT);
  else if (!cmd.is("?")) Serial.println(F("# unknown command"));
  anglePid.setTunings(akp, aki, 0.0f);
  ratePid.setTunings(rkp, rki, rkd);
  printSettings();
}

void setup() {
  Serial.begin(115200);
  safety.begin();
  anglePid.begin();
  ratePid.setDerivativeFilter(FILTER_EMA, 0.7f);
  ratePid.begin();
  Serial.println(F("# s07_capstone: cascaded controller + acceptance test ('test')"));
  if (imu.begin()) {
    imu.calibrateGyro();
  } else {
    Serial.println(F("# IMU NOT FOUND"));
  }
  printSettings();
  Serial.println(F("t_ms,test_t,sp,pot,est,rate_sp,rate,u,us1,us2,armed"));
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
  float pot = potAngle(counts);
  bool imuOk = imu.update(dtMs / 1000.0f);
  bool armed = safety.update(counts, pot, stop || !imuOk);
  if (!armed) testActive = false;

  float ts = testActive ? (now - testStart) / 1000.0f : -1.0f;
  float faultEta = 1.0f, dist = 0.0f;
  if (testActive) {
    if (ts >= TEST_END_S) {
      testActive = false;
      setpoint = 0.0f;
      Serial.println(F("# TEST DONE"));
    } else {
      setpoint = testSetpoint(ts);
      if (ts >= DIST_TIME_S && ts < DIST_TIME_S + DIST_MS / 1000.0f) dist = DIST_US;
      if (ts >= FAULT_TIME_S) faultEta = FAULT_ETA;
    }
  }
  float est = useImu ? imu.angle() : pot;
  teamSchedule(setpoint, est);

  float rateSp = 0.0f, u = 0.0f, m1 = ESC_US_MIN, m2 = ESC_US_MIN;
  if (armed) {
    float r = safety.ramp();
    if (r < 1.0f) {
      m1 = m2 = US_IDLE + r * (baseUs - US_IDLE);
      wasRamping = true;
    } else {
      if (wasRamping) {
        anglePid.reset();
        ratePid.reset();
        wasRamping = false;
      }
      rateSp = anglePid.update(setpoint, est, dtMs);
      u = ratePid.update(rateSp, imu.rate(), dtMs) + teamFeedforward(setpoint, est) + dist;
      mixDifferential(baseUs, u, US_IDLE, US_CAP, m1, m2);
      m2 = US_IDLE + faultEta * (m2 - US_IDLE);
    }
  } else {
    wasRamping = true;
  }
  safety.writeMotors(m1, m2);

  if (++logDivider >= 4) {
    logDivider = 0;
    Serial.print(now);
    Serial.print(',');
    Serial.print(ts, 3);
    Serial.print(',');
    Serial.print(setpoint, 1);
    Serial.print(',');
    Serial.print(pot, 2);
    Serial.print(',');
    Serial.print(est, 2);
    Serial.print(',');
    Serial.print(rateSp, 1);
    Serial.print(',');
    Serial.print(imu.rate(), 1);
    Serial.print(',');
    Serial.print(u, 1);
    Serial.print(',');
    Serial.print(armed ? m1 : ESC_US_MIN, 0);
    Serial.print(',');
    Serial.print(armed ? m2 : ESC_US_MIN, 0);
    Serial.print(',');
    Serial.println(armed ? 1 : 0);
  }
}
