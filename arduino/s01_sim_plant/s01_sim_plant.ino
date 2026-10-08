// s01_sim_plant -- Lab 1, part A: easyPID on an Arduino with a *simulated* plant.
// Drone-control bootcamp, session 1 (sessions/01-feedback-and-pid/lab.md). MIT licence.
//
// No wiring at all: the board simulates a first-order plant  tau y' + y = K u  and
// closes the loop around it with easyPID, 50 times a second. Open the Serial Plotter
// (115200 baud) and change things live by typing commands:
//
//   kp 2.0   ki 1.0   kd 0.05   gains           sp 60     setpoint
//   tau 0.5  gain 1.2            plant           dist 20   input disturbance
//   reset    clear the controller and the plant  ?         print the settings
//
// Output (CSV, one line per sample): t_ms,sp,y,u,p,i,d
#include <easyPID.h>

#include "serial_cmd.h"

const unsigned long PERIOD_MS = 20;  // 50 Hz
const float U_MIN = 0.0f;
const float U_MAX = 100.0f;  // think "percent of actuator range"

float kp = 1.0f, ki = 0.0f, kd = 0.0f;
float setpoint = 50.0f;
float plantTau = 1.0f;   // s
float plantGain = 1.5f;  // plant output per unit input: ceiling 150 at u = 100
float disturbance = 0.0f;
float y = 0.0f;

PIDController pid(kp, ki, kd, U_MIN, U_MAX);
SerialCommands cmd;
unsigned long lastMs = 0;

void printSettings() {
  Serial.print(F("# kp="));
  Serial.print(kp, 3);
  Serial.print(F(" ki="));
  Serial.print(ki, 3);
  Serial.print(F(" kd="));
  Serial.print(kd, 3);
  Serial.print(F(" sp="));
  Serial.print(setpoint, 1);
  Serial.print(F(" tau="));
  Serial.print(plantTau, 3);
  Serial.print(F(" gain="));
  Serial.print(plantGain, 3);
  Serial.print(F(" dist="));
  Serial.println(disturbance, 1);
}

void handleCommand() {
  if (cmd.is("kp")) kp = cmd.f(0, kp);
  else if (cmd.is("ki")) ki = cmd.f(0, ki);
  else if (cmd.is("kd")) kd = cmd.f(0, kd);
  else if (cmd.is("sp")) setpoint = cmd.f(0, setpoint);
  else if (cmd.is("tau") && cmd.f(0) > 0.0f) plantTau = cmd.f(0);
  else if (cmd.is("gain")) plantGain = cmd.f(0, plantGain);
  else if (cmd.is("dist")) disturbance = cmd.f(0, 0.0f);
  else if (cmd.is("reset")) {
    pid.reset();
    y = 0.0f;
  } else if (!cmd.is("?")) {
    Serial.println(F("# unknown command"));
  }
  pid.setTunings(kp, ki, kd);
  printSettings();
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("# s01_sim_plant: easyPID + simulated first-order plant"));
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

  // easyPID with a caller-supplied time step (milliseconds, not seconds)
  float u = pid.update(setpoint, y, dtMs);

  // Exact discretisation of tau y' + y = K (u + d) over one period, input held.
  float a = exp(-(dtMs / 1000.0f) / plantTau);
  y = a * y + (1.0f - a) * plantGain * (u + disturbance);

  Serial.print(now);
  Serial.print(',');
  Serial.print(setpoint, 2);
  Serial.print(',');
  Serial.print(y, 2);
  Serial.print(',');
  Serial.print(u, 2);
  Serial.print(',');
  Serial.print(pid.getPterm(), 2);
  Serial.print(',');
  Serial.print(pid.getIterm(), 2);
  Serial.print(',');
  Serial.println(pid.getDterm(), 2);
}
