// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "epuck_ros2_cpp/mpu9250.hpp"

#include <cmath>

#define MPU9250_ADDRESS_AD1_0 0x68
#define MPU9250_ADDRESS_AD1_1 0x69

#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

#define RAW2G (32768.0 / 2)
#define G2MS2 (9.81)
#define RAW2DEG (32768.0 / 250.0)

MPU9250::MPU9250(std::shared_ptr<I2CWrapper> i2c) {
  mI2c = i2c;
  mAddress = MPU9250_ADDRESS_AD1_0;
}

int MPU9250::read_register(char reg, char *data, int size) {
  if (mI2c->writeData(&reg, 1) != 1)
    return -1;

  if (mI2c->readData(data, size) != size)
    return -2;

  return 0;
}

void MPU9250::read_raw(int16_t *rawAccelerometer, int16_t *rawGyroscope) {
  int status;
  static char buffer[6];

  for (int i = 0; i < 5; i++) {
    status = mI2c->setAddress(mAddress);

    status ^= read_register(ACCEL_XOUT_H, buffer, 6);
    for (int i = 0; i < 3; i++)
      rawAccelerometer[i] = (buffer[i * 2 + 1] & 0x00FF) | ((buffer[i * 2] << 8) & 0xFF00);

    status ^= read_register(GYRO_XOUT_H, buffer, 6);
    for (int i = 0; i < 3; i++)
      rawGyroscope[i] = (buffer[i * 2 + 1] & 0x00FF) | ((buffer[i * 2] << 8) & 0xFF00);

    if (status)
      mAddress = (mAddress == MPU9250_ADDRESS_AD1_1) ? MPU9250_ADDRESS_AD1_0 : MPU9250_ADDRESS_AD1_1;
    else
      break;
  }
}

void MPU9250::calibrate() {
  read_raw(mOffsetAccelerometer, mOffsetGyroscope);
}

void MPU9250::read() {
  static int16_t rawGyroscope[3];
  static int16_t rawAccelerometer[3];

  // Read accelerometer and gyroscope data
  read_raw(rawAccelerometer, rawGyroscope);

  // Apply calibration
  for (int i = 0; i < 3; i++) {
    rawAccelerometer[i] -= mOffsetAccelerometer[i];
    rawGyroscope[i] -= mOffsetGyroscope[i];
  }

  // Scale data
  for (int i = 0; i < 3; i++) {
    mAccelerometer[i] = ((float)rawAccelerometer[i] / RAW2G) * G2MS2;
    mGyroscope[i] = ((float)rawGyroscope[i] / RAW2DEG) * (M_PI / 180);
  }
}

std::vector<float> MPU9250::getAngularVelocity() {
  std::vector<float> vector;
  vector.reserve(3);
  vector[0] = mGyroscope[0];
  vector[1] = mGyroscope[1];
  vector[2] = mGyroscope[2];
  return vector;
}

std::vector<float> MPU9250::getLinearAcceleration() {
  std::vector<float> vector;
  vector.reserve(3);
  vector[0] = mAccelerometer[0];
  vector[1] = mAccelerometer[1];
  vector[2] = mAccelerometer[2];
  return vector;
}