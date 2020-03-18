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

#include "epuck_ros2_cpp/pipuck_imu.h"

#include <assert.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <math.h>
#include <stdint.h>

#define MPU9250_ADDRESS_AD1_0 0x68
#define MPU9250_ADDRESS_AD1_1 0x69

#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

#define RAW2G (32768.0 / 2)
#define G2MS2 (9.81)
#define RAW2DEG (32768.0 / 250.0)

typedef struct _pipuck_imu_raw_t {
  int16_t gyroscope[3];
  int16_t accelerometer[3];
} pipuck_imu_raw_t;

static char buffer[6];
static pipuck_imu_raw_t imu_read;
static pipuck_imu_raw_t imu_offset;

static void select_device(int file);
static int read_register(int file, uint16_t reg, char *data, int size);

void select_device(int file) {
  int success;
  static int address = MPU9250_ADDRESS_AD1_0;

  for (int i = 0; i < 3; i++) {
    success = ioctl(file, I2C_SLAVE, address);
    if (success) {
      return;
    }
    address = (address == MPU9250_ADDRESS_AD1_1) ? MPU9250_ADDRESS_AD1_0 : MPU9250_ADDRESS_AD1_1;
  }
}

int read_register(int file, uint16_t reg, char *data, int size) {
  int success;
  if (write(file, &reg, 1) != 1) {
    return -1;
  }
  if (read(file, data, size) != size) {
    return -2;
  }
  return 0;
}

void read_raw(int file, pipuck_imu_raw_t *raw) {
  select_device(file);
  for (int i = 0; i < 3; i++) {
    read_register(file, ACCEL_XOUT_H, buffer, 6);
  }
  for (int i = 0; i < 3; i++) {
    raw->accelerometer[i] = (buffer[i * 2 + 1] & 0x00FF) | ((buffer[i * 2] << 8) & 0xFF00);
  }
  for (int i = 0; i < 3; i++) {
    read_register(file, GYRO_XOUT_H, buffer, 6);
  }
  for (int i = 0; i < 3; i++) {
    raw->gyroscope[i] = (buffer[i * 2 + 1] & 0x00FF) | ((buffer[i * 2] << 8) & 0xFF00);
  }
}

void pipuck_imu_calibrate(int file) {
  read_raw(file, &imu_offset);
}

void pipuck_imu_read(int file, pipuck_imu_t *data) {
  // Read accelerometer and gyroscope data
  read_raw(file, &imu_read);

  // Apply calibration
  for (int i = 0; i < 3; i++) {
    imu_read.accelerometer[i] -= imu_offset.accelerometer[i];
    imu_read.gyroscope[i] -= imu_offset.gyroscope[i];
  }

  // Scale data
  for (int i = 0; i < 3; i++) {
    data->accelerometer[i] = ((float)imu_read.accelerometer[i] / RAW2G) / G2MS2;
    data->gyroscope[i] = ((float)imu_read.gyroscope[i] / RAW2DEG) * (M_PI / 180);
  }
}