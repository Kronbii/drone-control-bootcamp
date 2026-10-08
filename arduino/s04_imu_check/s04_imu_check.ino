// s04_imu_check -- Lab 4, part A: what an IMU really measures, against the pot.
// Drone-control bootcamp, session 4 (sessions/04-rotations-and-attitude/lab.md). MIT.
//
// The pivot pot is the ground truth. The MPU-6050 gives three estimates of the same
// beam angle: accelerometer only, gyro integrated, and a complementary filter. Log them
// with the motors off, then with both motors at a constant throttle (vibration!).
//
// Wiring: as s03 plus MPU-6050 SDA -> A4, SCL -> A5, VCC -> 5V, GND -> GND,
// mounted on the pivot axis (hardware/rig/build.md). Keep the beam still for 2 s after
// reset: the gyro bias is measured then.
//
// Commands (115200 baud):
//   tau 0.5            complementary-filter time constant (s)
//   both 1300          both motors at a fixed pulse (armed with the switch, props guarded)
//   x                  stop motors         gcal   re-measure gyro bias (disarmed, still)
//   zero               record the pot reading as level
// Output (100 Hz): t_ms,pot,acc,gyro_int,comp,rate,us
#include "beam_imu.h"
#include "esc_timer1.h"
#include "mpu6050_lite.h"
#include "rig_safety.h"
#include "serial_cmd.h"

int potCenter = 512;
float potSign = 1.0f;
const unsigned long LOOP_US = 5000;  // 200 Hz filter update, 100 Hz log

BeamImu imu;
RigSafety safety;
SerialCommands cmd;
float motorUs = US_IDLE;
unsigned long lastLoop = 0;
bool logTick = false;

float potAngle(int counts) { return (counts - potCenter) * POT_DEG_PER_COUNT * potSign; }

void handleCommand(bool& stop) {
  if (cmd.is("x") || cmd.is("stop")) {
    stop = true;
  } else if (cmd.is("tau")) {
    imu.setTimeConstant(cmd.f(0, 0.5f));
  } else if (cmd.is("both")) {
    motorUs = constrain(cmd.f(0, US_IDLE), (float)US_IDLE, 1450.0f);
  } else if (cmd.is("zero")) {
    potCenter = analogRead(PIN_POT);
  } else if (cmd.is("gcal")) {
    if (!safety.armed()) imu.calibrateGyro();
  } else {
    Serial.println(F("# unknown command"));
    return;
  }
  Serial.print(F("# tau="));
  Serial.print(imu.timeConstant(), 3);
  Serial.print(F(" motors="));
  Serial.print(motorUs, 0);
  Serial.print(F(" gyro_bias="));
  Serial.println(imu.gyroBias(), 3);
}

void setup() {
  Serial.begin(115200);
  safety.begin();
  Serial.println(F("# s04_imu_check: pot vs accelerometer vs gyro vs complementary filter"));
  if (!imu.begin()) {
    Serial.println(F("# IMU NOT FOUND: check SDA->A4, SCL->A5, power, address 0x68"));
  } else {
    Serial.print(F("# WHO_AM_I = 0x"));
    Serial.println(imu.whoAmI(), HEX);
    Serial.println(F("# keep the beam still: measuring gyro bias"));
    imu.calibrateGyro();
    Serial.print(F("# gyro bias (deg/s) = "));
    Serial.println(imu.gyroBias(), 3);
  }
  Serial.println(F("t_ms,pot,acc,gyro_int,comp,rate,us"));
  lastLoop = micros();
}

void loop() {
  bool stop = false;
  if (cmd.poll()) handleCommand(stop);
  unsigned long nowUs = micros();
  if (nowUs - lastLoop < LOOP_US) return;
  float dt = (nowUs - lastLoop) * 1e-6f;
  lastLoop = nowUs;

  int counts = analogRead(PIN_POT);
  float pot = potAngle(counts);
  bool armed = safety.update(counts, pot, stop);
  imu.update(dt);
  float us = armed ? US_IDLE + safety.ramp() * (motorUs - US_IDLE) : ESC_US_MIN;
  safety.writeMotors(us, us);

  logTick = !logTick;
  if (logTick) {
    Serial.print(millis());
    Serial.print(',');
    Serial.print(pot, 2);
    Serial.print(',');
    Serial.print(imu.accelAngle(), 2);
    Serial.print(',');
    Serial.print(imu.gyroAngle(), 2);
    Serial.print(',');
    Serial.print(imu.angle(), 2);
    Serial.print(',');
    Serial.print(imu.rate(), 2);
    Serial.print(',');
    Serial.println(us, 0);
  }
}
