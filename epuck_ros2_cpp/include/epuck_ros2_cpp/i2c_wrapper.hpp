/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef EPUCK_ROS2_CPP__I2C_WRAPPER_HPP_
#define EPUCK_ROS2_CPP__I2C_WRAPPER_HPP_

extern "C"
{
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/stat.h>
}

#include <string>
#include <iostream>
#include <fstream>


class I2CWrapper
{
public:
  virtual int set_address(int address) = 0;
  virtual int read_data(char * buffer, int size) = 0;
  virtual int write_data(char * buffer, int size) = 0;
};

class I2CWrapperTest : public I2CWrapper
{
public:
  I2CWrapperTest() {}
  explicit I2CWrapperTest(std::string device)
  {
    base_filename = "/tmp" + device;

    // Create folder
    std::string folder = base_filename.substr(0, base_filename.find_last_of("/"));
    mkdir(folder.c_str(), 0777);
    std::cout << "Folder " << folder << " is created" << std::endl;
  }

  int set_address(int address)
  {
    std::fstream stream(base_filename + "_ioctl", std::ios::out | std::ios::binary);
    stream << address << std::endl;
    stream.close();
    return 1;
  }

  int read_data(char * buffer, int size)
  {
    std::fstream stream(base_filename + "_read", std::ios::in | std::ios::binary);
    stream.read(buffer, size);
    stream.close();
    return size;
  }

  int write_data(char * buffer, int size)
  {
    std::fstream stream;
    stream.open(base_filename + "_write", std::ios::out | std::ios::binary);

    stream.write(buffer, size);
    stream.close();
    return size;
  }

private:
  std::string base_filename;
};

class I2CWrapperHW : public I2CWrapper
{
public:
  I2CWrapperHW() {}

  explicit I2CWrapperHW(std::string device)
  {
    fh = open(device.c_str(), O_RDWR);
    std::cout << fh << std::endl;
    if (fh < 0) {
      std::cout << "Cannot open file: " << device << std::endl;
    }
  }

  int set_address(int address)
  {
    return ioctl(fh, I2C_SLAVE, address);
  }

  int read_data(char * buffer, int size)
  {
    return read(fh, buffer, size);
  }

  int write_data(char * buffer, int size)
  {
    return write(fh, buffer, size);
  }

private:
  int fh;
};

#endif  // EPUCK_ROS2_CPP__I2C_WRAPPER_HPP_
