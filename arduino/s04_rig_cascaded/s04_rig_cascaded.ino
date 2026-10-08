// s04_rig_cascaded -- Lab 4, part B: cascaded angle/rate control, like a flight controller.
// Drone-control bootcamp, session 4 (sessions/04-rotations-and-attitude/lab.md). MIT.
//
// Two easyPID instances (200 Hz):
//   outer  angle error (deg)        -> rate setpoint (deg/s)      P or PI, "a" gains
//   inner  rate error (deg/s, gyro) -> differential command (us)  PID,     "r" gains
// The outer loop can use the pot (ground truth) or the IMU complementary filter, which
// is what a real drone has. The safety trip always uses the pot.
//
// Commands (115200 baud):
//   akp 4  aki 0             outer gains       rkp 0.8  rki 0.6  rkd 0.01   inner gains
//   src pot | src imu        angle source       tau 0.5   complementary-filter tau (s)
//   sp 10 | seq              setpoint (deg) or the 0/+10/-10/0 step test
//   rmax 150                 rate-setpoint limit (deg/s)
//   fault 0.7 | fault off    motor-2 throttle scaling (fault injection, lab 6)
//   base 1350 | zero | x | ? base throttle, level calibration, disarm, settings
// Output (50 Hz): t_ms,sp,pot,est,rate_sp,rate,u,us1,us2,armed
#include <easyPID.h>

#include "beam_imu.h"
#include "esc_timer1.h"
#include "mpu6050_lite.h"
#include "rig_safety.h"
#include "serial_cmd.h"

int potCenter = 512;
float potSign = 1.0f;

const unsigned long LOOP_US = 5000;  // 200 Hz
const float U_LIMIT = 200.0f;        // us
float rateLimit = 150.0f;            // deg/s
float akp = 4.0f, aki = 0.0f;
float rkp = 0.8f, rki = 0.6f, rkd = 0.01f;
float baseUs = 1350.0f, setpoint = 0.0f, faultEta = 1.0f;
bool useImu = false;

PIDController anglePid(akp, aki, 0.0f, -rateLimit, rateLimit);
PIDController ratePid(rkp, rki, rkd, -U_LIMIT, U_LIMIT);
BeamImu imu;
RigSafety safety;
SerialCommands cmd;

bool seqActive = false, wasRamping = true;
unsigned long seqStart = 0, lastLoop = 0;
uint8_t logDivider = 0;

float potAngle(int counts) { return (counts - potCenter) * POT_DEG_PER_COUNT * potSign; }

void printSettings() {
  Serial.print(F("# angle: kp="));
  Serial.print(akp, 3);
  Serial.print(F(" ki="));
  Serial.print(aki, 3);
  Serial.print(F(" | rate: kp="));
  Serial.print(rkp, 3);
  Serial.print(F(" ki="));
  Serial.print(rki, 3);
  Serial.print(F(" kd="));
  Serial.print(rkd, 4);
  Serial.print(useImu ? F(" | src=imu tau=") : F(" | src=pot tau="));
  Serial.print(imu.timeConstant(), 2);
  Serial.print(F(" sp="));
  Serial.print(setpoint, 1);
  Serial.print(F(" fault_eta="));
  Serial.println(faultEta, 2);
}

void handleCommand(bool& stop) {
  if (cmd.is("x") || cmd.is("stop")) stop = true;
  else if (cmd.is("akp")) akp = cmd.f(0, akp);
  else if (cmd.is("aki")) aki = cmd.f(0, aki);
  else if (cmd.is("rkp")) rkp = cmd.f(0, rkp);
  else if (cmd.is("rki")) rki = cmd.f(0, rki);
  else if (cmd.is("rkd")) rkd = cmd.f(0, rkd);
  else if (cmd.is("src")) useImu = strcmp(cmd.s(0), "imu") == 0;
  else if (cmd.is("tau")) imu.setTimeConstant(cmd.f(0, 0.5f));
  else if (cmd.is("sp")) {
    setpoint = constrain(cmd.f(0, setpoint), -20.0f, 20.0f);
    seqActive = false;
  } else if (cmd.is("seq")) {
    seqActive = true;
    seqStart = millis();
  } else if (cmd.is("rmax")) {
    rateLimit = constrain(cmd.f(0, rateLimit), 20.0f, 400.0f);
    anglePid.setOutputLimits(-rateLimit, rateLimit);
  } else if (cmd.is("fault")) {
    faultEta = (strcmp(cmd.s(0), "off") == 0) ? 1.0f : constrain(cmd.f(0, 1.0f), 0.3f, 1.0f);
  } else if (cmd.is("base")) baseUs = constrain(cmd.f(0, baseUs), (float)US_IDLE, 1500.0f);
  else if (cmd.is("zero")) potCenter = analogRead(PIN_POT);
  else if (!cmd.is("?")) Serial.println(F("# unknown command"));
  anglePid.setTunings(akp, aki, 0.0f);
  ratePid.setTunings(rkp, rki, rkd);
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
  anglePid.begin();
  ratePid.setDerivativeFilter(FILTER_EMA, 0.7f);
  ratePid.begin();
  Serial.println(F("# s04_rig_cascaded: angle (outer) + rate (inner) easyPID loops"));
  if (imu.begin()) {
    Serial.println(F("# keep the beam still: measuring gyro bias"));
    imu.calibrateGyro();
  } else {
    Serial.println(F("# IMU NOT FOUND: the inner loop needs the gyro. Fix wiring first."));
  }
  printSettings();
  Serial.println(F("t_ms,sp,pot,est,rate_sp,rate,u,us1,us2,armed"));
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
  // Without a gyro the inner loop is blind: treat an IMU failure as a stop request.
  bool armed = safety.update(counts, pot, stop || !imuOk);
  if (seqActive) setpoint = sequenceSetpoint(now);
  float est = useImu ? imu.angle() : pot;

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
      u = ratePid.update(rateSp, imu.rate(), dtMs);
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
