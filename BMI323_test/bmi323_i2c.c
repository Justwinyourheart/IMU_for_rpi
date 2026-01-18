/*
 * bmi323_i2c.c
 *
 * Raspberry Pi C implementation for interfacing with the BMI323 6-axis IMU over I2C.
 * Reads accelerometer, gyroscope, and temperature data at 20Hz.
 * Outputs data in CSV format to stdout.
 *
 * Raspberry Pi I2C pinout (I2C1):
 *   GPIO2  -> SDA
 *   GPIO3  -> SCL
 *   GND    -> SDO (for address 0x69)
 *
 * Output Format: ax,ay,az,gx,gy,gz,temp
 * Units: g,g,g,deg/s,deg/s,deg/s,C
 *
 * Compile: make
 * Run: sudo ./bmi323_i2c
 */

#include "../common/common.h"

// I2C address and register definitions
#define BMI323_I2C_ADDR_1 0x69  // I2C address can be 0x68 or 0x69 if SDO = GND
#define CHIP_ID_REG 0x00
#define ERR_REG 0x01
#define STATUS_REG 0x02
#define ACC_CONF_REG 0x20
#define GYR_CONF_REG 0x21
#define ACC_DATA_X_REG 0x03
#define ACC_DATA_Y_REG 0x04
#define ACC_DATA_Z_REG 0x05
#define GYR_DATA_X_REG 0x06
#define GYR_DATA_Y_REG 0x07
#define GYR_DATA_Z_REG 0x08
#define TEMP_DATA_REG 0x09
#define CMD_REG 0x7E
#define FEATURE_IO0_REG 0x10
#define FEATURE_IO1_REG 0x11
#define FEATURE_IO2_REG 0x12
#define FEATURE_IO_STATUS_REG 0x14
#define FEATURE_CTRL_REG 0x40

// Configuration constants
#define OUTPUT_RATE_HZ 20  // 20Hz data output
#define ACC_RANGE_LSB_PER_G 4096.0  // +-8g range
#define GYR_RANGE_LSB_PER_DPS 16.384  // +-2000 deg/s range
#define SOFT_RESET_CMD 0xDEAF  // Soft reset command
#define ACC_CONF_NORMAL_100HZ_8G 0x4028  // Accel: 100Hz, +-8g
#define GYR_CONF_NORMAL_100HZ_2000DPS 0x4048  // Gyro: 100Hz, +-2000 deg/s

// Global variables
uint8_t bmi323_i2c_addr = BMI323_I2C_ADDR_1;

// Function prototypes
int init_bmi323(void);
int initialize_feature_engine(void);
float convert_accel_data(uint16_t raw_data);
float convert_gyro_data(uint16_t raw_data);
float convert_temp_data(uint16_t raw_data);

// Initialize feature engine
int initialize_feature_engine(void) {
    // Clear feature config
    i2c_write16(FEATURE_IO0_REG, 0x0000);
    delay_ms(1);

    // Set startup config
    i2c_write16(FEATURE_IO2_REG, 0x012C);
    delay_ms(1);

    // Trigger startup
    i2c_write16(FEATURE_IO_STATUS_REG, 0x0001);
    delay_ms(1);

    // Enable feature engine
    i2c_write16(FEATURE_CTRL_REG, 0x0001);
    delay_ms(10);

    // Check feature engine status
    int timeout = 0;
    uint16_t feature_io1_status;
    do {
        delay_ms(10);
        feature_io1_status = i2c_read16_with_dummy(FEATURE_IO1_REG);
        uint8_t error_status = feature_io1_status & 0x0F;
        if (error_status == 0x01) {
            printf("Feature engine initialized successfully\n");
            return 1;
        }
        if (error_status == 0x03) {
            fprintf(stderr, "Feature engine error\n");
            return 0;
        }
        timeout++;
    } while ((feature_io1_status & 0x0F) == 0x00 && timeout < 50);

    fprintf(stderr, "Feature engine timeout\n");
    return 0;
}

// Initialize BMI323
int init_bmi323(void) {
    // Try I2C address
    printf("Testing I2C address 0x%02X\n", BMI323_I2C_ADDR_1);

    if (i2c_init(I2C_DEVICE, BMI323_I2C_ADDR_1) < 0) {
        return 0;
    }

    uint16_t chip_id = i2c_read16_with_dummy(CHIP_ID_REG);
    if ((chip_id & 0xFF) == 0x43 || (chip_id & 0xFF) == 0x41) {
        bmi323_i2c_addr = BMI323_I2C_ADDR_1;
        printf("Found BMI323 at address 0x%02X\n", BMI323_I2C_ADDR_1);
    } else {
        fprintf(stderr, "Invalid chip ID: 0x%04X\n", chip_id);
        return 0;
    }

    // Reset sensor
    printf("Performing soft reset...\n");
    i2c_write16(CMD_REG, SOFT_RESET_CMD);
    delay_ms(5);

    // Set up feature engine
    printf("Initializing feature engine...\n");
    if (!initialize_feature_engine()) {
        fprintf(stderr, "Feature engine setup failed\n");
        return 0;
    }

    // Configure accelerometer and gyroscope
    printf("Configuring accelerometer and gyroscope...\n");
    i2c_write16(ACC_CONF_REG, ACC_CONF_NORMAL_100HZ_8G);
    i2c_write16(GYR_CONF_REG, GYR_CONF_NORMAL_100HZ_2000DPS);
    delay_ms(50);

    return 1;
}

// Convert accelerometer data to g
float convert_accel_data(uint16_t raw_data) {
    int16_t signed_data = (int16_t)raw_data;
    if (signed_data == -32768) return NAN;
    return signed_data / ACC_RANGE_LSB_PER_G;
}

// Convert gyroscope data to deg/s
float convert_gyro_data(uint16_t raw_data) {
    int16_t signed_data = (int16_t)raw_data;
    if (signed_data == -32768) return NAN;
    return signed_data / GYR_RANGE_LSB_PER_DPS;
}

// Convert temperature data to Celsius
float convert_temp_data(uint16_t raw_data) {
    int16_t signed_data = (int16_t)raw_data;
    if (signed_data == -32768) return NAN;
    return (signed_data / 512.0) + 23.0;
}

int main(void) {
    printf("BMI323 I2C Example for Raspberry Pi\n");

    delay_ms(200);  // Wait for sensor power-up

    if (!init_bmi323()) {
        fprintf(stderr, "BMI323 failed to initialize! Check wiring, SDO pin, and power supply.\n");
        i2c_close();
        return 1;
    }

    printf("BMI323 ready\n");
    printf("Format: ax,ay,az,gx,gy,gz,temp\n");
    printf("Units: g,g,g,deg/s,deg/s,deg/s,C\n");

    // Main loop
    const unsigned long print_interval = 1000 / OUTPUT_RATE_HZ;
    uint64_t last_print_time = 0;

    while (1) {
        // Limit output to 20Hz
        uint64_t current_time = millis();
        if (current_time - last_print_time < print_interval) {
            delay_ms(1);
            continue;
        }
        last_print_time = current_time;

        // Read sensor data
        uint16_t acc_x = i2c_read16_with_dummy(ACC_DATA_X_REG);
        uint16_t acc_y = i2c_read16_with_dummy(ACC_DATA_Y_REG);
        uint16_t acc_z = i2c_read16_with_dummy(ACC_DATA_Z_REG);
        uint16_t gyr_x = i2c_read16_with_dummy(GYR_DATA_X_REG);
        uint16_t gyr_y = i2c_read16_with_dummy(GYR_DATA_Y_REG);
        uint16_t gyr_z = i2c_read16_with_dummy(GYR_DATA_Z_REG);
        uint16_t temp_raw = i2c_read16_with_dummy(TEMP_DATA_REG);

        // Convert to physical units
        float ax = convert_accel_data(acc_x);
        float ay = convert_accel_data(acc_y);
        float az = convert_accel_data(acc_z);
        float gx = convert_gyro_data(gyr_x);
        float gy = convert_gyro_data(gyr_y);
        float gz = convert_gyro_data(gyr_z);
        float temp = convert_temp_data(temp_raw);

        // Print valid data in CSV format
        if (!isnan(ax) && !isnan(ay) && !isnan(az) &&
            !isnan(gx) && !isnan(gy) && !isnan(gz)) {
            printf("%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,", ax, ay, az, gx, gy, gz);
            if (isnan(temp)) {
                printf("NAN\n");
            } else {
                printf("%.1f\n", temp);
            }
            fflush(stdout);
        }
    }

    i2c_close();
    return 0;
}
