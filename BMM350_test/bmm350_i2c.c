/*
 * bmm350_i2c.c
 *
 * Raspberry Pi C implementation for interfacing with the BMM350 3-axis magnetometer over I2C.
 * Reads magnetometer (X, Y, Z) and temperature data at 20Hz.
 * Outputs data in CSV format to stdout.
 *
 * Raspberry Pi I2C pinout (I2C1):
 *   GPIO2  -> SDA
 *   GPIO3  -> SCL
 *   GND    -> GND
 *
 * Output Format: mx,my,mz,temp
 * Units: uT,uT,uT,C
 *
 * Compile: make
 * Run: sudo ./bmm350_i2c
 * Calibration: sudo ./bmm350_i2c -c [duration_sec]
 */

#include "../common/common.h"

// I2C address (depends on CSB pin: 0x14 if CSB=GND, 0x15 if CSB=VDD)
#define BMM350_I2C_ADDR_PRIMARY   0x14
#define BMM350_I2C_ADDR_SECONDARY 0x15

// Register addresses
#define BMM350_REG_CHIP_ID        0x00
#define BMM350_REG_REV_ID         0x01
#define BMM350_REG_ERR_REG        0x02
#define BMM350_REG_PAD_CTRL       0x03
#define BMM350_REG_PMU_CMD_AGGR   0x04
#define BMM350_REG_PMU_CMD_AXIS   0x05
#define BMM350_REG_PMU_CMD        0x06
#define BMM350_REG_PMU_CMD_STATUS 0x07
#define BMM350_REG_I2C_WDT        0x0A
#define BMM350_REG_INT_CTRL       0x2E
#define BMM350_REG_INT_STATUS     0x30
#define BMM350_REG_MAG_X_XLSB     0x31
#define BMM350_REG_TEMP_XLSB      0x3A
#define BMM350_REG_SENSORTIME_XLSB 0x3D
#define BMM350_REG_OTP_CMD        0x50
#define BMM350_REG_OTP_DATA_MSB   0x52
#define BMM350_REG_OTP_DATA_LSB   0x53
#define BMM350_REG_OTP_STATUS     0x55
#define BMM350_REG_CMD            0x7E

// Chip ID
#define BMM350_CHIP_ID            0x33

// PMU commands
#define BMM350_PMU_CMD_SUSPEND    0x00
#define BMM350_PMU_CMD_NORMAL     0x01
#define BMM350_PMU_CMD_UPD_OAE    0x02
#define BMM350_PMU_CMD_FORCED     0x03
#define BMM350_PMU_CMD_FORCED_FAST 0x04
#define BMM350_PMU_CMD_FGR        0x05
#define BMM350_PMU_CMD_FGR_FAST   0x06
#define BMM350_PMU_CMD_BR         0x07
#define BMM350_PMU_CMD_BR_FAST    0x08

// OTP commands
#define BMM350_OTP_CMD_DIR_READ   0x20
#define BMM350_OTP_CMD_PWR_OFF    0x80
#define BMM350_OTP_STATUS_ERROR   0xE0

// ODR settings (Output Data Rate)
#define BMM350_ODR_400HZ          0x02
#define BMM350_ODR_200HZ          0x03
#define BMM350_ODR_100HZ          0x04
#define BMM350_ODR_50HZ           0x05
#define BMM350_ODR_25HZ           0x06
#define BMM350_ODR_12_5HZ         0x07
#define BMM350_ODR_6_25HZ         0x08
#define BMM350_ODR_3_125HZ        0x09
#define BMM350_ODR_1_5625HZ       0x0A

// Averaging settings
#define BMM350_AVG_NO_AVG         0x00
#define BMM350_AVG_2              0x01
#define BMM350_AVG_4              0x02
#define BMM350_AVG_8              0x03

// Soft reset command
#define BMM350_CMD_SOFTRESET      0xB6

// Enable all axes
#define BMM350_AXIS_EN_X          0x01
#define BMM350_AXIS_EN_Y          0x02
#define BMM350_AXIS_EN_Z          0x04
#define BMM350_AXIS_EN_ALL        (BMM350_AXIS_EN_X | BMM350_AXIS_EN_Y | BMM350_AXIS_EN_Z)

// Configuration constants
#define OUTPUT_RATE_HZ            20   // 20Hz data output
#define BMM350_OTP_DATA_LENGTH    32

// LSB to uT conversion factor (from datasheet)
// For 21-bit data shifted to 24-bit container
#define BMM350_LSB_TO_UT          0.0625f

// User calibration file and defaults
#define USER_CAL_FILENAME "bmm350_cal.txt"
#define DEFAULT_CAL_DURATION 30

// Global variables
uint8_t bmm350_i2c_addr = BMM350_I2C_ADDR_PRIMARY;
uint16_t otp_data[BMM350_OTP_DATA_LENGTH];

// Calibration data structure (factory OTP)
typedef struct {
    float offset_x;
    float offset_y;
    float offset_z;
    float sens_x;
    float sens_y;
    float sens_z;
    float tco_x;
    float tco_y;
    float tco_z;
    float tcs_x;
    float tcs_y;
    float tcs_z;
    float t0;
    float cross_x_y;
    float cross_y_x;
    float cross_z_x;
    float cross_z_y;
} bmm350_cal_t;

bmm350_cal_t cal_data;

// User calibration data structure (hard iron / soft iron)
typedef struct {
    float offset_x;
    float offset_y;
    float offset_z;
    float scale_x;
    float scale_y;
    float scale_z;
    int valid;
} user_cal_t;

user_cal_t user_cal = {0, 0, 0, 1, 1, 1, 0};

// Function prototypes
int init_bmm350(void);
int otp_dump(void);
void update_calibration(void);
int magnetic_reset(void);
int read_mag_temp_data(float *mx, float *my, float *mz, float *temp);
int32_t fix_sign(uint32_t val, uint8_t bits);
int load_user_calibration(const char *filename);
int save_user_calibration(const char *filename);
int run_calibration(int duration_sec);
void print_usage(const char *progname);

// Sign extend from N bits to 32-bit signed
int32_t fix_sign(uint32_t val, uint8_t bits) {
    int32_t result;
    uint32_t sign_bit = 1U << (bits - 1);

    if (val & sign_bit) {
        // Negative: extend sign
        result = (int32_t)(val | (~0U << bits));
    } else {
        result = (int32_t)val;
    }
    return result;
}

// Read OTP word at specified address
int read_otp_word(uint8_t addr, uint16_t *word) {
    uint8_t otp_cmd = BMM350_OTP_CMD_DIR_READ | (addr & 0x1F);

    // Write OTP command with address
    if (i2c_write(BMM350_REG_OTP_CMD, otp_cmd) < 0) {
        return -1;
    }

    // Wait for OTP read to complete
    delay_us(300);

    // Check OTP status
    uint8_t status;
    if (i2c_read_with_dummy(BMM350_REG_OTP_STATUS, &status, 1) < 0) {
        return -1;
    }

    if (status & BMM350_OTP_STATUS_ERROR) {
        fprintf(stderr, "OTP error at address 0x%02X, status: 0x%02X\n", addr, status);
        return -1;
    }

    // Read MSB and LSB
    uint8_t data[2];
    if (i2c_read_with_dummy(BMM350_REG_OTP_DATA_MSB, &data[0], 1) < 0) {
        return -1;
    }
    if (i2c_read_with_dummy(BMM350_REG_OTP_DATA_LSB, &data[1], 1) < 0) {
        return -1;
    }

    *word = ((uint16_t)data[0] << 8) | data[1];
    return 0;
}

// Dump all OTP data
int otp_dump(void) {
    printf("Reading OTP calibration data...\n");

    for (int i = 0; i < BMM350_OTP_DATA_LENGTH; i++) {
        if (read_otp_word(i, &otp_data[i]) < 0) {
            fprintf(stderr, "Failed to read OTP word %d\n", i);
            return -1;
        }
    }

    // Power off OTP
    if (i2c_write(BMM350_REG_OTP_CMD, BMM350_OTP_CMD_PWR_OFF) < 0) {
        return -1;
    }
    delay_us(400);

    printf("OTP dump complete\n");
    return 0;
}

// Extract and calculate calibration coefficients from OTP data
void update_calibration(void) {
    // Offset coefficients (indices 0x0E-0x10)
    int16_t off_x = (int16_t)((otp_data[0x0E] & 0x0FFF) << 4) >> 4;  // 12-bit signed
    int16_t off_y = (int16_t)((otp_data[0x0F] & 0x0FFF) << 4) >> 4;
    int16_t off_z = (int16_t)((otp_data[0x10] & 0x0FFF) << 4) >> 4;

    cal_data.offset_x = (float)off_x;
    cal_data.offset_y = (float)off_y;
    cal_data.offset_z = (float)off_z;

    // Temperature offset (index 0x0D)
    int8_t t_off = (int8_t)(otp_data[0x0D] & 0xFF);
    cal_data.t0 = ((float)t_off / 5.0f);

    // Sensitivity coefficients (indices 0x10-0x11)
    int8_t sens_x = (int8_t)((otp_data[0x10] >> 8) & 0xFF);
    int8_t sens_y = (int8_t)(otp_data[0x11] & 0xFF);
    int8_t sens_z = (int8_t)((otp_data[0x11] >> 8) & 0xFF);

    cal_data.sens_x = (float)sens_x / 256.0f;
    cal_data.sens_y = (float)sens_y / 256.0f + 0.001f;  // Y has small correction
    cal_data.sens_z = (float)sens_z / 256.0f;

    // TCO coefficients (index 0x12-0x13)
    int8_t tco_x = (int8_t)(otp_data[0x12] & 0xFF);
    int8_t tco_y = (int8_t)((otp_data[0x12] >> 8) & 0xFF);
    int8_t tco_z = (int8_t)(otp_data[0x13] & 0xFF);

    cal_data.tco_x = (float)tco_x / 32.0f;
    cal_data.tco_y = (float)tco_y / 32.0f;
    cal_data.tco_z = (float)tco_z / 32.0f;

    // TCS coefficients (index 0x13-0x14)
    int8_t tcs_x = (int8_t)((otp_data[0x13] >> 8) & 0xFF);
    int8_t tcs_y = (int8_t)(otp_data[0x14] & 0xFF);
    int8_t tcs_z = (int8_t)((otp_data[0x14] >> 8) & 0xFF);

    cal_data.tcs_x = (float)tcs_x / 16384.0f;
    cal_data.tcs_y = (float)tcs_y / 16384.0f;
    cal_data.tcs_z = (float)tcs_z / 16384.0f;

    // Cross-axis coefficients (indices 0x15-0x16)
    int8_t c_xy = (int8_t)(otp_data[0x15] & 0xFF);
    int8_t c_yx = (int8_t)((otp_data[0x15] >> 8) & 0xFF);
    int8_t c_zx = (int8_t)(otp_data[0x16] & 0xFF);
    int8_t c_zy = (int8_t)((otp_data[0x16] >> 8) & 0xFF);

    cal_data.cross_x_y = (float)c_xy / 800.0f;
    cal_data.cross_y_x = (float)c_yx / 800.0f;
    cal_data.cross_z_x = (float)c_zx / 800.0f;
    cal_data.cross_z_y = (float)c_zy / 800.0f;

    printf("Calibration data loaded\n");
}

// Perform magnetic reset (flux guide reset)
int magnetic_reset(void) {
    printf("Performing magnetic reset (FGR)...\n");

    // Send FGR command
    if (i2c_write(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_FGR) < 0) {
        return -1;
    }

    // Wait for FGR to complete (40ms typical)
    delay_ms(40);

    // Send BR (bit reset) command for better accuracy
    if (i2c_write(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_BR) < 0) {
        return -1;
    }
    delay_ms(14);

    printf("Magnetic reset complete\n");
    return 0;
}

// Read all magnetic and temperature data in one transaction
int read_mag_temp_data(float *mx, float *my, float *mz, float *temp) {
    // Read 12 bytes: X(3), Y(3), Z(3), Temp(3)
    uint8_t data[12];

    if (i2c_read_with_dummy(BMM350_REG_MAG_X_XLSB, data, 12) < 0) {
        return -1;
    }

    // Extract 24-bit values (little endian)
    uint32_t raw_x = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
    uint32_t raw_y = (uint32_t)data[3] | ((uint32_t)data[4] << 8) | ((uint32_t)data[5] << 16);
    uint32_t raw_z = (uint32_t)data[6] | ((uint32_t)data[7] << 8) | ((uint32_t)data[8] << 16);
    uint32_t raw_t = (uint32_t)data[9] | ((uint32_t)data[10] << 8) | ((uint32_t)data[11] << 16);

    // Data is 21-bit signed, left-justified in 24-bit container
    // Shift right by 3 to get actual 21-bit value
    int32_t mag_x = fix_sign(raw_x >> 3, 21);
    int32_t mag_y = fix_sign(raw_y >> 3, 21);
    int32_t mag_z = fix_sign(raw_z >> 3, 21);
    int32_t temp_raw = fix_sign(raw_t >> 3, 21);

    // Convert temperature
    float t_comp = ((float)temp_raw / 65536.0f) + 23.0f;
    float t_delta = t_comp - cal_data.t0 - 25.0f;  // Delta from reference

    // Apply temperature compensation and sensitivity to magnetic data
    // Simple conversion to uT with basic compensation
    float x_raw_ut = (float)mag_x * BMM350_LSB_TO_UT;
    float y_raw_ut = (float)mag_y * BMM350_LSB_TO_UT;
    float z_raw_ut = (float)mag_z * BMM350_LSB_TO_UT;

    // Apply offset correction
    x_raw_ut -= cal_data.offset_x * BMM350_LSB_TO_UT;
    y_raw_ut -= cal_data.offset_y * BMM350_LSB_TO_UT;
    z_raw_ut -= cal_data.offset_z * BMM350_LSB_TO_UT;

    // Apply sensitivity correction
    x_raw_ut *= (1.0f + cal_data.sens_x);
    y_raw_ut *= (1.0f + cal_data.sens_y);
    z_raw_ut *= (1.0f + cal_data.sens_z);

    // Apply temperature coefficient correction
    x_raw_ut += cal_data.tco_x * t_delta;
    y_raw_ut += cal_data.tco_y * t_delta;
    z_raw_ut += cal_data.tco_z * t_delta;

    // Apply cross-axis correction
    *mx = x_raw_ut + cal_data.cross_x_y * y_raw_ut;
    *my = y_raw_ut + cal_data.cross_y_x * x_raw_ut;
    *mz = z_raw_ut + cal_data.cross_z_x * x_raw_ut + cal_data.cross_z_y * y_raw_ut;
    *temp = t_comp;

    // Apply user calibration (hard iron / soft iron) if loaded
    if (user_cal.valid) {
        *mx = (*mx - user_cal.offset_x) * user_cal.scale_x;
        *my = (*my - user_cal.offset_y) * user_cal.scale_y;
        *mz = (*mz - user_cal.offset_z) * user_cal.scale_z;
    }

    return 0;
}

// Load user calibration from file
int load_user_calibration(const char *filename) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        return 0;  // File not found, not an error
    }

    char line[256];
    int found = 0;

    while (fgets(line, sizeof(line), fp)) {
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '\n') continue;

        if (sscanf(line, "offset_x=%f", &user_cal.offset_x) == 1) found++;
        else if (sscanf(line, "offset_y=%f", &user_cal.offset_y) == 1) found++;
        else if (sscanf(line, "offset_z=%f", &user_cal.offset_z) == 1) found++;
        else if (sscanf(line, "scale_x=%f", &user_cal.scale_x) == 1) found++;
        else if (sscanf(line, "scale_y=%f", &user_cal.scale_y) == 1) found++;
        else if (sscanf(line, "scale_z=%f", &user_cal.scale_z) == 1) found++;
    }

    fclose(fp);

    if (found >= 6) {
        user_cal.valid = 1;
        printf("User calibration loaded from %s\n", filename);
        printf("  Offsets: X=%.2f Y=%.2f Z=%.2f uT\n",
               user_cal.offset_x, user_cal.offset_y, user_cal.offset_z);
        printf("  Scales:  X=%.3f Y=%.3f Z=%.3f\n",
               user_cal.scale_x, user_cal.scale_y, user_cal.scale_z);
        return 1;
    }

    return 0;
}

// Save user calibration to file
int save_user_calibration(const char *filename) {
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        fprintf(stderr, "Error: Cannot create calibration file %s\n", filename);
        return -1;
    }

    fprintf(fp, "# BMM350 User Calibration (Hard Iron / Soft Iron)\n");
    fprintf(fp, "# Generated by bmm350_i2c calibration\n");
    fprintf(fp, "offset_x=%.4f\n", user_cal.offset_x);
    fprintf(fp, "offset_y=%.4f\n", user_cal.offset_y);
    fprintf(fp, "offset_z=%.4f\n", user_cal.offset_z);
    fprintf(fp, "scale_x=%.6f\n", user_cal.scale_x);
    fprintf(fp, "scale_y=%.6f\n", user_cal.scale_y);
    fprintf(fp, "scale_z=%.6f\n", user_cal.scale_z);

    fclose(fp);
    printf("Calibration saved to %s\n", filename);
    return 0;
}

// Run calibration routine
int run_calibration(int duration_sec) {
    printf("\n=== BMM350 Calibration Mode ===\n");
    printf("Duration: %d seconds\n", duration_sec);
    printf("Rotate the sensor slowly in ALL directions (figure-8 pattern)\n");
    printf("Try to cover all orientations: roll, pitch, and yaw\n");
    printf("Starting in 3 seconds...\n\n");

    delay_ms(3000);

    float min_x = 1e9, max_x = -1e9;
    float min_y = 1e9, max_y = -1e9;
    float min_z = 1e9, max_z = -1e9;

    uint64_t start_time = millis();
    uint64_t end_time = start_time + (duration_sec * 1000);
    uint64_t last_print = 0;
    int samples = 0;

    // Temporarily disable user calibration during calibration
    int was_valid = user_cal.valid;
    user_cal.valid = 0;

    printf("Collecting data... (rotate the sensor!)\n");

    while (millis() < end_time) {
        float mx, my, mz, temp;

        if (read_mag_temp_data(&mx, &my, &mz, &temp) == 0) {

            //reject spurious
            if(fabs(mx) > 1000 || fabs(my) > 1000 || fabs(mz) > 1000)
                continue;

            // Update min/max
            if (mx < min_x) min_x = mx;
            if (mx > max_x) max_x = mx;
            if (my < min_y) min_y = my;
            if (my > max_y) max_y = my;
            if (mz < min_z) min_z = mz;
            if (mz > max_z) max_z = mz;
            samples++;
            printf("min: %.2f,%.2f,%.2f    max: %.2f,%.2f,%.2f\n", min_x, min_y, min_z, max_x, max_y, max_z);

        }

        // Progress update every second
        uint64_t now = millis();
        if (now - last_print >= 1000) {
            int remaining = (int)((end_time - now) / 1000);
            printf("\r  %d seconds remaining... (%d samples)   ", remaining, samples);
            fflush(stdout);
            last_print = now;
        }

        delay_ms(20);  // ~50Hz sampling
    }

    printf("\n\nCalibration complete! %d samples collected.\n\n", samples);

    // Restore previous state
    user_cal.valid = was_valid;

    if (samples < 100) {
        fprintf(stderr, "Error: Not enough samples collected. Check sensor connection.\n");
        return -1;
    }

    // Calculate hard iron offsets (center of the sphere)
    user_cal.offset_x = (max_x + min_x) / 2.0f;
    user_cal.offset_y = (max_y + min_y) / 2.0f;
    user_cal.offset_z = (max_z + min_z) / 2.0f;

    // Calculate soft iron scales (normalize to sphere)
    float range_x = (max_x - min_x) / 2.0f;
    float range_y = (max_y - min_y) / 2.0f;
    float range_z = (max_z - min_z) / 2.0f;
    float avg_range = (range_x + range_y + range_z) / 3.0f;

    if (avg_range < 1.0f) {
        fprintf(stderr, "Error: Insufficient magnetic field range. Move sensor more.\n");
        return -1;
    }

    user_cal.scale_x = avg_range / range_x;
    user_cal.scale_y = avg_range / range_y;
    user_cal.scale_z = avg_range / range_z;
    user_cal.valid = 1;

    // Print results
    printf("Measured ranges:\n");
    printf("  X: %.2f to %.2f uT (range: %.2f)\n", min_x, max_x, range_x * 2);
    printf("  Y: %.2f to %.2f uT (range: %.2f)\n", min_y, max_y, range_y * 2);
    printf("  Z: %.2f to %.2f uT (range: %.2f)\n", min_z, max_z, range_z * 2);
    printf("\nCalculated calibration:\n");
    printf("  Hard Iron Offsets: X=%.2f Y=%.2f Z=%.2f uT\n",
           user_cal.offset_x, user_cal.offset_y, user_cal.offset_z);
    printf("  Soft Iron Scales:  X=%.3f Y=%.3f Z=%.3f\n",
           user_cal.scale_x, user_cal.scale_y, user_cal.scale_z);

    // Save to file
    if (save_user_calibration(USER_CAL_FILENAME) < 0) {
        return -1;
    }

    printf("\nCalibration complete!\n");
    return 0;
}

// Print usage
void print_usage(const char *progname) {
    printf("Usage: %s [options]\n", progname);
    printf("Options:\n");
    printf("  -c, --calibrate [sec]  Run calibration mode (default: %d seconds)\n", DEFAULT_CAL_DURATION);
    printf("  -h, --help             Show this help message\n");
    printf("\nNormal mode reads magnetometer data at 20Hz.\n");
    printf("Calibration file (%s) is loaded automatically if present.\n", USER_CAL_FILENAME);
}

// Initialize BMM350
int init_bmm350(void) {
    uint8_t chip_id;

    // Try primary I2C address
    printf("Testing I2C address 0x%02X\n", BMM350_I2C_ADDR_PRIMARY);

    if (i2c_init(I2C_DEVICE, BMM350_I2C_ADDR_PRIMARY) < 0) {
        // Try secondary address
        printf("Testing I2C address 0x%02X\n", BMM350_I2C_ADDR_SECONDARY);
        if (i2c_init(I2C_DEVICE, BMM350_I2C_ADDR_SECONDARY) < 0) {
            return 0;
        }
        bmm350_i2c_addr = BMM350_I2C_ADDR_SECONDARY;
    } else {
        bmm350_i2c_addr = BMM350_I2C_ADDR_PRIMARY;
    }

    // Read and verify chip ID
    if (i2c_read_with_dummy(BMM350_REG_CHIP_ID, &chip_id, 1) < 0) {
        fprintf(stderr, "Failed to read chip ID\n");
        // Try secondary address
        i2c_close();
        printf("Testing I2C address 0x%02X\n", BMM350_I2C_ADDR_SECONDARY);
        if (i2c_init(I2C_DEVICE, BMM350_I2C_ADDR_SECONDARY) < 0) {
            return 0;
        }
        bmm350_i2c_addr = BMM350_I2C_ADDR_SECONDARY;
        if (i2c_read_with_dummy(BMM350_REG_CHIP_ID, &chip_id, 1) < 0) {
            fprintf(stderr, "Failed to read chip ID at secondary address\n");
            return 0;
        }
    }

    if (chip_id != BMM350_CHIP_ID) {
        fprintf(stderr, "Invalid chip ID: 0x%02X (expected 0x%02X)\n", chip_id, BMM350_CHIP_ID);
        return 0;
    }

    printf("Found BMM350 at address 0x%02X (Chip ID: 0x%02X)\n", bmm350_i2c_addr, chip_id);

    // Perform soft reset
    printf("Performing soft reset...\n");
    i2c_write(BMM350_REG_CMD, BMM350_CMD_SOFTRESET);
    delay_ms(24);  // Wait for reset to complete

    // Re-init I2C after reset (reset clears I2C state machine)
    i2c_close();
    delay_ms(3);  // Power-up delay
    if (i2c_init(I2C_DEVICE, bmm350_i2c_addr) < 0) {
        return 0;
    }

    // Verify chip ID after reset
    if (i2c_read_with_dummy(BMM350_REG_CHIP_ID, &chip_id, 1) < 0) {
        fprintf(stderr, "Failed to read chip ID after reset\n");
        return 0;
    }

    if (chip_id != BMM350_CHIP_ID) {
        fprintf(stderr, "Chip ID mismatch after reset: 0x%02X\n", chip_id);
        return 0;
    }

    // Dump OTP calibration data
    if (otp_dump() < 0) {
        fprintf(stderr, "Failed to read OTP data\n");
        return 0;
    }

    // Parse calibration coefficients
    update_calibration();

    // Perform magnetic reset
    if (magnetic_reset() < 0) {
        fprintf(stderr, "Magnetic reset failed\n");
        return 0;
    }

    // Enable all axes (X, Y, Z)
    printf("Enabling all magnetic axes...\n");
    if (i2c_write(BMM350_REG_PMU_CMD_AXIS, BMM350_AXIS_EN_ALL) < 0) {
        return 0;
    }
    delay_ms(1);

    // Configure ODR (100Hz) and averaging (4 samples)
    printf("Configuring ODR and averaging...\n");
    uint8_t aggr_set = (BMM350_AVG_4 << 4) | BMM350_ODR_100HZ;
    if (i2c_write(BMM350_REG_PMU_CMD_AGGR, aggr_set) < 0) {
        return 0;
    }
    delay_ms(1);

    // Set to Normal mode
    printf("Setting Normal mode...\n");
    if (i2c_write(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_NORMAL) < 0) {
        return 0;
    }
    delay_ms(50);  // Wait for mode transition and first measurement

    // Verify PMU status
    uint8_t pmu_status;
    if (i2c_read_with_dummy(BMM350_REG_PMU_CMD_STATUS, &pmu_status, 1) < 0) {
        fprintf(stderr, "Failed to read PMU status\n");
        return 0;
    }
    printf("PMU Status: 0x%02X\n", pmu_status);

    return 1;
}

int main(int argc, char *argv[]) {
    int calibrate_mode = 0;
    int cal_duration = DEFAULT_CAL_DURATION;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--calibrate") == 0) {
            calibrate_mode = 1;
            // Check if duration is specified
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                cal_duration = atoi(argv[i + 1]);
                if (cal_duration < 5) cal_duration = 5;
                if (cal_duration > 300) cal_duration = 300;
                i++;
            }
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    printf("BMM350 I2C Example for Raspberry Pi\n");

    delay_ms(200);  // Wait for sensor power-up

    if (!init_bmm350()) {
        fprintf(stderr, "BMM350 failed to initialize! Check wiring and power supply.\n");
        i2c_close();
        return 1;
    }

    printf("BMM350 ready\n");

    // Calibration mode
    if (calibrate_mode) {
        int result = run_calibration(cal_duration);
        i2c_close();
        return (result < 0) ? 1 : 0;
    }

    // Normal mode: load user calibration if available
    load_user_calibration(USER_CAL_FILENAME);

    printf("Format: mx,my,mz,temp\n");
    printf("Units: uT,uT,uT,C\n");

    // Main loop
    const unsigned long print_interval = 1000 / OUTPUT_RATE_HZ;
    uint64_t last_print_time = 0;

    while (1) {
        uint64_t current_time = millis();
        if (current_time - last_print_time < print_interval) {
            delay_ms(1);
            continue;
        }
        last_print_time = current_time;

        float mx, my, mz, temp;

        if (read_mag_temp_data(&mx, &my, &mz, &temp) == 0) {
            printf("%.2f,%.2f,%.2f,%.1f\n", mx, my, mz, temp);
            fflush(stdout);
        }
    }

    i2c_close();
    return 0;
}
