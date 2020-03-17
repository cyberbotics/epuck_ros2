// This file is taken from:
// https://github.com/gctronic/Pi-puck/blob/master/camera-configuration.c

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/i2c-dev.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define I2C_CHANNEL 4
#define OV7670_ADDR 0x21

/*######################
## OV7670 registers ##
######################
*/
#define REG_CLKRC 0x11  // Clocl control
#define REG_COM7 0x12   // Control 7
#define REG_COM10 0x15  // Control 10
#define COM7_FMT_VGA 0x00
#define COM7_YUV 0x00      // YUV
#define COM7_RGB 0x04      // bits 0 and 2 - RGB format
#define REG_TSLB 0x3a      // lots of stuff
#define REG_COM11 0x3b     // Control 11
#define REG_COM15 0x40     // Control 15
#define COM15_R00FF 0xc0   // 00 to FF
#define COM15_RGB565 0x10  // RGB565 output
#define REG_HSTART 0x17    // Horiz start high bits
#define REG_HSTOP 0x18     // Horiz stop high bits
#define REG_VSTART 0x19    // Vert start high bits
#define REG_VSTOP 0x1a     // Vert stop high bits
#define REG_VREF 0x03      // Pieces of GAIN, VSTART, VSTOP
#define REG_HREF 0x32      // HREF pieces
#define REG_RGB444 0x8c    // RGB 444 control
#define REG_COM9 0x14      // Control 9  - gain ceiling
#define REG_COM13 0x3d     // Control 13
#define COM13_GAMMA 0x80   // Gamma enable
#define COM13_UVSAT 0x40   // UV saturation auto adjustment

int file = -1;
char filename[20] = {0};
uint8_t data[2];
uint32_t sensor_id = 0;

static int write_i2c(uint8_t reg, uint8_t val);
static int read_i2c(uint8_t reg, uint8_t *val);

int write_i2c(uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  int trials = 0;
  while (trials < 5) {
    if (write(file, &buf, 2) != 2) {
      // printf("Failed writing register 0x%02x!\n", reg);
      trials++;
      usleep(200000);
    } else
      break;
  }
  if (trials == 5)
    return -1;
  return 0;
}

int read_i2c(uint8_t reg, uint8_t *val) {
  int trials = 0;
  while (trials < 5) {
    if (write(file, &reg, 1) != 1) {
      // printf("Failed writing register 0x%02x!\n", reg);
      trials++;
      usleep(200000);
    } else
      break;
  }
  if (trials == 5)
    return -1;
  trials = 0;
  while (trials < 5) {
    if (read(file, val, 1) != 1) {
      // printf("Failed reading 0x%02x!\n", *val);
      trials++;
      usleep(200000);
    } else
      break;
  }
  if (trials == 5)
    return -1;
  return 0;
}

int pipuck_ov7670_init(void) {
  sprintf(filename, "/dev/i2c-%d", 4);
  if ((file = open(filename, O_RDWR)) < 0) {
    /* ERROR HANDLING: you can check errno to see what went wrong */
    perror("Failed to open the i2c bus");
    exit(1);
  }

  /* open device to communicate with */
  // if (ioctl(file, I2C_SLAVE_FORCE, OV7670_ADDR) < 0) {
  if (ioctl(file, I2C_SLAVE, OV7670_ADDR) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    printf("Error setting slave device\n");
    close(file);
    exit(1);
  }

  if (read_i2c(0x0a, &data[0]) < 0)
    return -1;
  if (read_i2c(0x0b, &data[1]) < 0)
    return -1;

  if (write_i2c(REG_COM7, 0x80) < 0)  // Reset to default values.
    return -1;
  usleep(200000);
  if (write_i2c(REG_CLKRC, 0x80) < 0)  // No internal clock prescaler.
    return -1;
  if (write_i2c(REG_TSLB, 0x04) < 0)  // 0x04 = 0x0c (default) but with YUYV
    return -1;
  if (write_i2c(REG_COM7, COM7_YUV | COM7_FMT_VGA) < 0)  // Output format: YUV, VGA.
    return -1;
  if (write_i2c(REG_COM15, COM15_R00FF) < 0)
    return -1;
  if (write_i2c(REG_COM13, 0x00) < 0)  // YUYV
    return -1;
  if (write_i2c(0xb0, 0x84) < 0)  // Color mode?? (Not documented!)
    return -1;

  if (write_i2c(REG_HSTART, 0x13) < 0)  // start = HSTART<<3 + HREF[2:0] = 19*8 + 6 = 158
    return -1;
  if (write_i2c(REG_HSTOP, 0x01) < 0)  // stop = HSTOP<<3 + HREF[5:3] = 1*8 + 6 = 14 (158+640-784)
    return -1;
  if (write_i2c(REG_HREF, 0x36) <
      0)  // With flag "edge offset" set, then the image is strange (too much clear, not sharp); so clear this bit.
    return -1;
  if (write_i2c(REG_VSTART, 0x02) < 0)  // start = VSTART<<2 + VREF[1:0] = 2*4 + 2 = 10
    return -1;
  if (write_i2c(REG_VSTOP, 0x7a) < 0)  // stop = VSTOP<<2 + VREF[3:2] = 122*4 + 2 = 490
    return -1;
  if (write_i2c(REG_VREF, 0x0a) < 0)
    return -1;

  // Output array is 784x510 => 21'000'000 / (784x510x2) = about 26 fps
  // To lower the framerate to 15 fps we insert dummy pixels and dummy rows: 21'000'000 / [(784+91)x(510+290)x2] = 15 fps
  if (write_i2c(0x2a, 0x00) < 0)  // Dummy pixels MSB
    return -1;
  if (write_i2c(0x2b, 0x5B) < 0)  // Dummy pixels LSB
    return -1;
  if (write_i2c(0x92, 0x22) < 0)  // Dummy rows LSB
    return -1;
  if (write_i2c(0x93, 0x01) < 0)  // Dummy rows LSB
    return -1;

  if (write_i2c(0x0c, 0x00) < 0)
    return -1;
  if (write_i2c(0x3e, 0x00) < 0)
    return -1;
  if (write_i2c(0x70, 0x3a) < 0)
    return -1;
  if (write_i2c(0x71, 0x35) < 0)
    return -1;
  if (write_i2c(0x72, 0x11) < 0)
    return -1;
  if (write_i2c(0x73, 0xf0) < 0)
    return -1;
  if (write_i2c(0xa2, 0x02) < 0)
    return -1;

  return 0;
}
