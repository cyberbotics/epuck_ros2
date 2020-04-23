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

#ifndef EPUCK_ROS2_DRIVER__I2C_WRAPPER_HPP_
#define EPUCK_ROS2_DRIVER__I2C_WRAPPER_HPP_

extern "C" {
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
}

#include <fstream>
#include <iostream>
#include <string>

class I2CWrapper {
public:
  virtual int setAddress(int address) = 0;
  virtual int readData(char *buffer, int size) = 0;
  virtual int writeData(char *buffer, int size) = 0;

  char readInt8Register(char reg) {
    int status;
    char byte;

    for (int i = 0; i < mMaxRetryCount; i++) {
      status = (this->writeData(&reg, 1) != 1);
      status ^= (this->readData(&byte, 1) != 1);

      if (!status)
        return byte;
    }
    return 0;
  }

protected:
  int mMaxRetryCount = 3;
};

class I2CWrapperTest : public I2CWrapper {
public:
  I2CWrapperTest() = delete;

  explicit I2CWrapperTest(std::string device) {
    mBaseFilename = "/tmp" + device;

    // Create folder
    std::string folder = mBaseFilename.substr(0, mBaseFilename.find_last_of("/"));
    mkdir(folder.c_str(), 0777);
    std::cout << "Folder " << folder << " is created" << std::endl;
  }

  int setAddress(int address) {
    mAddress = address;
    return 1;
  }

  int readData(char *buffer, int size) {
    std::fstream stream(mBaseFilename + "_read_" + std::to_string(mAddress), std::ios::in | std::ios::binary);
    stream.read(buffer, size);
    stream.close();
    return size;
  }

  int writeData(char *buffer, int size) {
    std::fstream stream;
    stream.open(mBaseFilename + "_write_" + std::to_string(mAddress), std::ios::out | std::ios::binary);

    stream.write(buffer, size);
    stream.close();
    return size;
  }

private:
  std::string mBaseFilename;
  int mAddress;
};

class I2CWrapperHW : public I2CWrapper {
public:
  I2CWrapperHW() = delete;

  explicit I2CWrapperHW(std::string device) {
    mFile = open(device.c_str(), O_RDWR);
    if (mFile < 0)
      std::cout << "Cannot open file: " << device << std::endl;
  }

  int setAddress(int address) { return ioctl(mFile, I2C_SLAVE, address); }

  int readData(char *buffer, int size) { return read(mFile, buffer, size); }

  int writeData(char *buffer, int size) { return write(mFile, buffer, size); }

private:
  int mFile;
};

#endif  // EPUCK_ROS2_DRIVER__I2C_WRAPPER_HPP_
