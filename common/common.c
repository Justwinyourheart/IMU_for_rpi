/*
 * common.c
 *
 * Common I2C and utility functions for Bosch sensor drivers on Raspberry Pi.
 */

#include "common.h"

// Global I2C file descriptor
int i2c_fd = -1;

// Initialize I2C
int i2c_init(const char *device, uint8_t addr) {
    i2c_fd = open(device, O_RDWR);
    if (i2c_fd < 0) {
        fprintf(stderr, "Error opening I2C device %s: %s\n", device, strerror(errno));
        return -1;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        fprintf(stderr, "Error setting I2C slave address 0x%02X: %s\n", addr, strerror(errno));
        close(i2c_fd);
        i2c_fd = -1;
        return -1;
    }

    return 0;
}

// Close I2C
void i2c_close(void) {
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }
}

// Normal read (no dummy bytes) - for BMP390
int i2c_read(uint8_t reg, uint8_t *data, uint8_t len) {
    // Write register address
    if (write(i2c_fd, &reg, 1) != 1) {
        fprintf(stderr, "I2C write error (reg 0x%02X): %s\n", reg, strerror(errno));
        return -1;
    }

    // Read data
    if (read(i2c_fd, data, len) != len) {
        fprintf(stderr, "I2C read error: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}

// Read with 2 dummy bytes - for BMI323, BMM350
int i2c_read_with_dummy(uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t buffer[len + 2];

    // Write register address
    if (write(i2c_fd, &reg, 1) != 1) {
        fprintf(stderr, "I2C write error (reg 0x%02X): %s\n", reg, strerror(errno));
        return -1;
    }

    // Small delay for I2C timing
    delay_us(100);

    // Read data with 2 dummy bytes
    ssize_t bytes_read = read(i2c_fd, buffer, len + 2);
    if (bytes_read != (ssize_t)(len + 2)) {
        fprintf(stderr, "I2C read error: expected %d bytes, got %zd: %s\n",
                len + 2, bytes_read, strerror(errno));
        return -1;
    }

    // Skip 2 dummy bytes and copy actual data
    memcpy(data, buffer + 2, len);
    return 0;
}

// Read 16-bit register with 2 dummy bytes - for BMI323
uint16_t i2c_read16_with_dummy(uint8_t reg) {
    uint8_t buffer[4];

    // Write register address
    if (write(i2c_fd, &reg, 1) != 1) {
        fprintf(stderr, "I2C write error: %s\n", strerror(errno));
        return 0xFFFF;
    }

    // Read 4 bytes: 2 dummy bytes + 2 data bytes (LSB, MSB)
    if (read(i2c_fd, buffer, 4) != 4) {
        fprintf(stderr, "I2C read error: Not enough bytes: %s\n", strerror(errno));
        return 0xFFFF;
    }

    // Discard dummy bytes (index 0 and 1), return data bytes
    uint8_t lsb = buffer[2];
    uint8_t msb = buffer[3];
    return (msb << 8) | lsb;
}

// Normal write (1 byte data) - for BMM350, BMP390
int i2c_write(uint8_t reg, uint8_t data) {
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = data;

    if (write(i2c_fd, buffer, 2) != 2) {
        fprintf(stderr, "I2C write error (reg 0x%02X): %s\n", reg, strerror(errno));
        return -1;
    }
    return 0;
}

// Write 16-bit data - for BMI323
int i2c_write16(uint8_t reg, uint16_t data) {
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = data & 0xFF;        // LSB
    buffer[2] = (data >> 8) & 0xFF; // MSB

    if (write(i2c_fd, buffer, 3) != 3) {
        fprintf(stderr, "I2C write error (reg 0x%02X): %s\n", reg, strerror(errno));
        return -1;
    }
    return 0;
}

// Get milliseconds since start
uint64_t millis(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

// Delay in milliseconds
void delay_ms(unsigned int ms) {
    usleep(ms * 1000);
}

// Delay in microseconds
void delay_us(unsigned int us) {
    usleep(us);
}
