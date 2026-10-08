// beam_imu.h -- beam angle and rate from an MPU-6050, with a complementary filter.
// Part of the drone-control bootcamp (MIT). Master copy in arduino/common/.
//
// Mounting assumed (hardware/rig/build.md): IMU flat on the beam, ON the pivot axis,
// x axis along the beam towards motor 1, z axis up. Then, with theta > 0 when the
// motor-1 end is up:
//   accelerometer at rest:  a_x = sin(theta), a_z = cos(theta)  ->  theta = atan2(a_x, a_z)
//   gyro:                   rotation is about -y                ->  rate = -g_y
// If your board is mounted differently, change BEAM_GYRO_SIGN / the axes below and
// check in the lab: tilt the beam by hand, pot and IMU angles must rise together.
#pragma once

#include <Arduino.h>

#include "mpu6050_lite.h"

const float BEAM_GYRO_SIGN = -1.0f;

class BeamImu {
 public:
  bool begin() {
    ok_ = imu_.begin(0x68);
    return ok_;
  }

  // Average the gyro while the beam is still (motors stopped) to remove its bias.
  void calibrateGyro(uint16_t samples = 400) {
    float sum = 0.0f;
    uint16_t n = 0;
    ImuSample s;
    for (uint16_t i = 0; i < samples; i++) {
      if (imu_.read(s)) {
        sum += BEAM_GYRO_SIGN * s.gy;
        n++;
      }
      delay(2);
    }
    bias_ = n ? sum / n : 0.0f;
  }

  // Read once per control period; dt in seconds. Returns false on a bus error.
  bool update(float dt) {
    ImuSample s;
    if (!ok_ || !imu_.read(s)) return false;
    rate_ = BEAM_GYRO_SIGN * s.gy - bias_;
    accAngle_ = atan2(s.ax, s.az) * RAD_TO_DEG;
    gyroAngle_ += rate_ * dt;  // pure integration: drifts (for comparison only)
    if (!primed_) {
      angle_ = accAngle_;
      gyroAngle_ = accAngle_;
      primed_ = true;
    }
    float alpha = tau_ / (tau_ + dt);
    angle_ = alpha * (angle_ + rate_ * dt) + (1.0f - alpha) * accAngle_;
    return true;
  }

  void setTimeConstant(float tauSeconds) { tau_ = tauSeconds > 0.0f ? tauSeconds : 0.0f; }
  float timeConstant() const { return tau_; }
  float angle() const { return angle_; }          // complementary filter (deg)
  float rate() const { return rate_; }            // bias-corrected gyro (deg/s)
  float accelAngle() const { return accAngle_; }  // accelerometer only (deg)
  float gyroAngle() const { return gyroAngle_; }  // gyro only (deg)
  float gyroBias() const { return bias_; }
  bool ok() const { return ok_; }
  uint8_t whoAmI() { return imu_.whoAmI(); }

 private:
  Mpu6050Lite imu_;
  bool ok_ = false, primed_ = false;
  float bias_ = 0.0f, rate_ = 0.0f, accAngle_ = 0.0f, gyroAngle_ = 0.0f, angle_ = 0.0f;
  float tau_ = 0.5f;
};
