// mpu6050_lite.h -- register-level MPU-6050 driver (no external library needed).
// Part of the drone-control bootcamp (MIT). Master copy in arduino/common/.
//
// Wiring (Uno/Nano): VCC -> 5V (the common GY-521 board has a regulator), GND -> GND,
// SDA -> A4, SCL -> A5. Address 0x68 (AD0 low) or 0x69 (AD0 high).
// Configuration: gyro +-500 deg/s (65.5 LSB per deg/s), accelerometer +-4 g
// (8192 LSB per g), digital low-pass filter about 44 Hz.
#pragma once

#include <Arduino.h>
#include <Wire.h>

struct ImuSample {
  float ax, ay, az;  // specific force (g)
  float gx, gy, gz;  // angular rate (deg/s)
};

class Mpu6050Lite {
 public:
  // Returns false if nothing acknowledges on the bus. whoAmI() tells clones apart
  // (0x68 genuine MPU-6050; 0x70 MPU-6500; other values: clones that usually work).
  bool begin(uint8_t address = 0x68) {
    addr_ = address;
    Wire.begin();
    Wire.setClock(400000UL);
    if (!writeReg(0x6B, 0x01)) return false;  // PWR_MGMT_1: wake up, PLL on gyro X
    delay(10);
    writeReg(0x1A, 0x03);  // CONFIG: DLPF_CFG = 3 (~44 Hz)
    writeReg(0x1B, 0x08);  // GYRO_CONFIG: +-500 deg/s
    writeReg(0x1C, 0x08);  // ACCEL_CONFIG: +-4 g
    delay(50);
    return true;
  }

  uint8_t whoAmI() {
    uint8_t v = 0;
    readRegs(0x75, &v, 1);
    return v;
  }

  bool read(ImuSample& s) {
    uint8_t b[14];
    if (!readRegs(0x3B, b, 14)) return false;
    s.ax = word16(b[0], b[1]) / 8192.0f;
    s.ay = word16(b[2], b[3]) / 8192.0f;
    s.az = word16(b[4], b[5]) / 8192.0f;
    // b[6..7] is the temperature
    s.gx = word16(b[8], b[9]) / 65.5f;
    s.gy = word16(b[10], b[11]) / 65.5f;
    s.gz = word16(b[12], b[13]) / 65.5f;
    return true;
  }

 private:
  static int16_t word16(uint8_t hi, uint8_t lo) { return (int16_t)(((uint16_t)hi << 8) | lo); }

  bool writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }

  bool readRegs(uint8_t reg, uint8_t* dst, uint8_t n) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr_, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) dst[i] = (uint8_t)Wire.read();
    return true;
  }

  uint8_t addr_ = 0x68;
};
