/*
 * common.h
 *
 * Common I2C and utility functions for Bosch sensor drivers on Raspberry Pi.
 */

#ifndef COMMON_H
#define COMMON_H

#define _POSIX_C_SOURCE 199309L
#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <errno.h>

// I2C device path
#define I2C_DEVICE "/dev/i2c-1"

// Global I2C file descriptor
extern int i2c_fd;

// I2C initialization and cleanup
int i2c_init(const char *device, uint8_t addr);
void i2c_close(void);

// I2C read functions
// Normal read (no dummy bytes) - for BMP390
int i2c_read(uint8_t reg, uint8_t *data, uint8_t len);

// Read with 2 dummy bytes - for BMI323, BMM350
int i2c_read_with_dummy(uint8_t reg, uint8_t *data, uint8_t len);

// Read 16-bit register with 2 dummy bytes - for BMI323
uint16_t i2c_read16_with_dummy(uint8_t reg);

// I2C write functions
// Normal write (1 byte data) - for BMM350, BMP390
int i2c_write(uint8_t reg, uint8_t data);

// Write 16-bit data - for BMI323
int i2c_write16(uint8_t reg, uint16_t data);

// Timing utilities
uint64_t millis(void);
void delay_ms(unsigned int ms);
void delay_us(unsigned int us);

#endif // COMMON_H
