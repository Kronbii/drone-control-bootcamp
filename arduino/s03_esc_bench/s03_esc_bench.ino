// s03_esc_bench -- Lab 3: ESC check, motor direction and thrust curve on the seesaw rig.
// Drone-control bootcamp, session 3 (sessions/03-the-1dof-rig/lab.md). MIT licence.
//
// READ hardware/rig/safety.md FIRST. Steps 1-3 of the lab are done with the PROPS OFF.
//
// Thrust curve without a thrust stand: rest the motor-2 end of the beam on a kitchen
// scale (beam level), tare it, and spin motor 1 only. With equal arms the scale reading
// rises by exactly the thrust of motor 1 (moment balance about the pivot).
//
// Wiring: ESC1 signal -> D9, ESC2 signal -> D10, ESC signal grounds -> GND,
// pivot pot wiper -> A0 (ends to 5V/GND), arm switch D2 -> GND, LED on D13.
// The ESCs are powered from the 12 V supply THROUGH the emergency-stop switch.
//
// Commands (115200 baud): arm with the switch (OFF -> ON), then
//   m1 1200 | m2 1200     pulse (us) for one motor; the other stays stopped
//   both 1200             both motors
//   sweep 1|2             motor 1 or 2: 1100 -> 1600 us in 50 us steps, 5 s per step;
//                          write down the scale reading at each "# STEP" line
//   x                     stop now (also: arm switch off, e-stop)
//   cal props-off         ESC throttle-range calibration (only some ESCs need it)
//   zero                  record the pot reading as the level position
// Output: t_ms,us1,us2,pot_deg
#include "esc_timer1.h"
#include "rig_safety.h"
#include "serial_cmd.h"

RigSafety safety;
SerialCommands cmd;

int potCenter = 512;
float potSign = 1.0f;
uint16_t us1 = ESC_US_MIN, us2 = ESC_US_MIN;
int8_t sweepMotor = 0;  // 0 = no sweep, 1 or 2
uint16_t sweepUs = 0;
unsigned long sweepNext = 0, lastLog = 0;
const uint16_t SWEEP_FROM = 1100, SWEEP_TO = 1600, SWEEP_STEP = 50;
const unsigned long SWEEP_HOLD_MS = 5000;

float potAngle(int counts) { return (counts - potCenter) * POT_DEG_PER_COUNT * potSign; }

uint16_t capped(float us) {
  if (us < ESC_US_MIN) return ESC_US_MIN;
  if (us > US_CAP) return US_CAP;
  return (uint16_t)us;
}

void calibrateEscs() {
  // Classic throttle-range calibration: max signal at ESC power-up, then min.
  Serial.println(F("# CALIBRATION: props must be OFF. Turn the 12 V e-stop OFF now."));
  Serial.println(F("# Sending 2000 us. Within 10 s: power the ESCs (e-stop ON), wait for"));
  Serial.println(F("# the 'max' beeps; the sketch then sends 1000 us for the 'min' beeps."));
  escWriteUs(0, ESC_US_MAX);
  escWriteUs(1, ESC_US_MAX);
  delay(10000);
  escStopAll();
  delay(4000);
  Serial.println(F("# calibration done; power-cycle the ESCs"));
}

void handleCommand(bool& stop) {
  if (cmd.is("x") || cmd.is("stop")) {
    stop = true;
    sweepMotor = 0;
  } else if (cmd.is("zero")) {
    potCenter = analogRead(PIN_POT);
    Serial.print(F("# POT_CENTER = "));
    Serial.println(potCenter);
  } else if (cmd.is("cal")) {
    if (strcmp(cmd.s(0), "props-off") == 0 && !safety.armed()) calibrateEscs();
    else Serial.println(F("# type 'cal props-off' while DISARMED"));
  } else if (!safety.armed()) {
    Serial.println(F("# not armed: flip the arm switch OFF then ON"));
  } else if (cmd.is("m1")) {
    us1 = capped(cmd.f(0, ESC_US_MIN));
    us2 = ESC_US_MIN;
  } else if (cmd.is("m2")) {
    us2 = capped(cmd.f(0, ESC_US_MIN));
    us1 = ESC_US_MIN;
  } else if (cmd.is("both")) {
    us1 = us2 = capped(cmd.f(0, ESC_US_MIN));
  } else if (cmd.is("sweep")) {
    sweepMotor = (int8_t)cmd.f(0, 1);
    sweepUs = SWEEP_FROM;
    sweepNext = millis();
  } else {
    Serial.println(F("# unknown command"));
  }
}

void setup() {
  Serial.begin(115200);
  safety.begin();
  Serial.println(F("# s03_esc_bench: ESC check and thrust curve. PROPS OFF until step 4."));
  Serial.println(F("t_ms,us1,us2,pot_deg"));
}

void loop() {
  bool stop = false;
  if (cmd.poll()) handleCommand(stop);
  int counts = analogRead(PIN_POT);
  float angle = potAngle(counts);
  // On the bench the beam rests on the scale, so only the switch, the pot wire and the
  // stop command are checked: pass 0 as the angle.
  bool armed = safety.update(counts, 0.0f, stop);
  unsigned long now = millis();

  if (!armed) {
    us1 = us2 = ESC_US_MIN;
    sweepMotor = 0;
  } else if (sweepMotor != 0 && (long)(now - sweepNext) >= 0) {
    if (sweepUs > SWEEP_TO) {
      sweepMotor = 0;
      us1 = us2 = ESC_US_MIN;
      Serial.println(F("# SWEEP DONE"));
    } else {
      us1 = (sweepMotor == 1) ? sweepUs : ESC_US_MIN;
      us2 = (sweepMotor == 2) ? sweepUs : ESC_US_MIN;
      Serial.print(F("# STEP "));
      Serial.print(sweepUs);
      Serial.println(F(" us: read the scale after 3 s"));
      sweepUs += SWEEP_STEP;
      sweepNext = now + SWEEP_HOLD_MS;
    }
  }
  if (armed) {
    escWriteUs(0, us1);
    escWriteUs(1, us2);
  } else {
    escStopAll();
  }

  if (now - lastLog >= 50) {
    lastLog = now;
    Serial.print(now);
    Serial.print(',');
    Serial.print(us1);
    Serial.print(',');
    Serial.print(us2);
    Serial.print(',');
    Serial.println(angle, 2);
  }
}
