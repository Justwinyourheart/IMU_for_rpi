/*
 * bmp390_i2c.c
 *
 * Raspberry Pi C implementation for interfacing with the BMP390 pressure sensor over I2C.
 * Reads pressure and temperature data at 20Hz.
 * Outputs data in CSV format to stdout.
 *
 * Raspberry Pi I2C pinout (I2C1):
 *   GPIO2  -> SDA
 *   GPIO3  -> SCL
 *   GND    -> GND
 *
 * Output Format: pressure,temperature,altitude
 * Units: Pa,C,m
 *
 * Compile: make
 * Run: sudo ./bmp390_i2c [options]
 *
 * Options:
 *   -q <qnh>      Set QNH (sea level pressure) in hPa (e.g., -q 1013.25)
 *   -a <altitude> Calibrate using known altitude in meters (e.g., -a 150)
 *   -h            Show help
 */

#include "../common/common.h"
#include <getopt.h>

// I2C addresses
#define BMP390_I2C_ADDR_PRIMARY   0x77
#define BMP390_I2C_ADDR_SECONDARY 0x76

// Register addresses
#define BMP390_REG_CHIP_ID        0x00
#define BMP390_REG_REV_ID         0x01
#define BMP390_REG_ERR            0x02
#define BMP390_REG_STATUS         0x03
#define BMP390_REG_DATA           0x04
#define BMP390_REG_EVENT          0x10
#define BMP390_REG_INT_STATUS     0x11
#define BMP390_REG_FIFO_LENGTH    0x12
#define BMP390_REG_FIFO_DATA      0x14
#define BMP390_REG_FIFO_WM        0x15
#define BMP390_REG_FIFO_CONFIG1   0x17
#define BMP390_REG_FIFO_CONFIG2   0x18
#define BMP390_REG_INT_CTRL       0x19
#define BMP390_REG_IF_CONF        0x1A
#define BMP390_REG_PWR_CTRL       0x1B
#define BMP390_REG_OSR            0x1C
#define BMP390_REG_ODR            0x1D
#define BMP390_REG_CONFIG         0x1F
#define BMP390_REG_CALIB_DATA     0x31
#define BMP390_REG_CMD            0x7E

// Chip IDs
#define BMP388_CHIP_ID            0x50
#define BMP390_CHIP_ID            0x60

// Commands
#define BMP390_CMD_SOFTRESET      0xB6

// Power modes
#define BMP390_MODE_SLEEP         0x00
#define BMP390_MODE_FORCED        0x01
#define BMP390_MODE_NORMAL        0x03

// Power control bits
#define BMP390_PRESS_EN           0x01
#define BMP390_TEMP_EN            0x02

// Oversampling settings
#define BMP390_OSR_NONE           0x00
#define BMP390_OSR_X1             0x00  // Same as NONE
#define BMP390_OSR_X2             0x01
#define BMP390_OSR_X4             0x02
#define BMP390_OSR_X8             0x03
#define BMP390_OSR_X16            0x04
#define BMP390_OSR_X32            0x05

// ODR settings
#define BMP390_ODR_200_HZ         0x00
#define BMP390_ODR_100_HZ         0x01
#define BMP390_ODR_50_HZ          0x02
#define BMP390_ODR_25_HZ          0x03
#define BMP390_ODR_12_5_HZ        0x04
#define BMP390_ODR_6_25_HZ        0x05
#define BMP390_ODR_3_1_HZ         0x06
#define BMP390_ODR_1_5_HZ         0x07
#define BMP390_ODR_0_78_HZ        0x08
#define BMP390_ODR_0_39_HZ        0x09
#define BMP390_ODR_0_2_HZ         0x0A
#define BMP390_ODR_0_1_HZ         0x0B
#define BMP390_ODR_0_05_HZ        0x0C
#define BMP390_ODR_0_02_HZ        0x0D
#define BMP390_ODR_0_01_HZ        0x0E
#define BMP390_ODR_0_006_HZ       0x0F
#define BMP390_ODR_0_003_HZ       0x10
#define BMP390_ODR_0_001_HZ       0x11

// Configuration constants
#define OUTPUT_RATE_HZ            20
#define BMP390_CALIB_DATA_LEN     21

// Default sea level pressure (Pa) - standard atmosphere
#define DEFAULT_SEA_LEVEL_PRESSURE 101325.0

// Global variables
uint8_t bmp390_i2c_addr = BMP390_I2C_ADDR_PRIMARY;
uint8_t chip_id_found = 0;

// Reference pressure for altitude calculation (can be calibrated)
double sea_level_pressure = DEFAULT_SEA_LEVEL_PRESSURE;

// Calibration mode
int calibrate_with_altitude = 0;
double known_altitude = 0.0;

// Calibration data structure (floating point version)
typedef struct {
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;  // Linearized temperature for pressure compensation
} bmp390_calib_t;

bmp390_calib_t calib;

// Function prototypes
int init_bmp390(void);
int read_calib_data(void);
void parse_calib_data(uint8_t *reg_data);
int read_sensor_data(double *pressure, double *temperature);
double compensate_temperature(uint32_t uncomp_temp);
double compensate_pressure(uint32_t uncomp_press);
double calculate_altitude(double pressure);
double calculate_sea_level_pressure(double pressure, double altitude);
void calibrate_altitude(void);
void print_usage(const char *program_name);

// Print usage/help
void print_usage(const char *program_name) {
    printf("Usage: %s [options]\n", program_name);
    printf("\nOptions:\n");
    printf("  -q <qnh>      Set QNH (sea level pressure) in hPa\n");
    printf("                Example: -q 1013.25 (standard atmosphere)\n");
    printf("                Get local QNH from weather service or airport METAR\n");
    printf("\n");
    printf("  -a <altitude> Calibrate using known altitude in meters\n");
    printf("                Example: -a 150 (if you know you're at 150m)\n");
    printf("                This calculates QNH from current pressure reading\n");
    printf("\n");
    printf("  -h            Show this help message\n");
    printf("\nOutput format: pressure(Pa),temperature(C),altitude(m)\n");
    printf("\nExamples:\n");
    printf("  %s                  # Use standard pressure (101325 Pa)\n", program_name);
    printf("  %s -q 1018.5        # Use local QNH of 1018.5 hPa\n", program_name);
    printf("  %s -a 42            # Calibrate: I'm at 42m altitude\n", program_name);
}

// Parse calibration data from raw bytes
void parse_calib_data(uint8_t *reg_data) {
    // Temperature calibration coefficients
    uint16_t par_t1_raw = (uint16_t)(reg_data[1] << 8) | reg_data[0];
    uint16_t par_t2_raw = (uint16_t)(reg_data[3] << 8) | reg_data[2];
    int8_t par_t3_raw = (int8_t)reg_data[4];

    calib.par_t1 = (double)par_t1_raw / pow(2, -8);  // = par_t1_raw * 256
    calib.par_t2 = (double)par_t2_raw / pow(2, 30);
    calib.par_t3 = (double)par_t3_raw / pow(2, 48);

    // Pressure calibration coefficients
    int16_t par_p1_raw = (int16_t)((reg_data[6] << 8) | reg_data[5]);
    int16_t par_p2_raw = (int16_t)((reg_data[8] << 8) | reg_data[7]);
    int8_t par_p3_raw = (int8_t)reg_data[9];
    int8_t par_p4_raw = (int8_t)reg_data[10];
    uint16_t par_p5_raw = (uint16_t)(reg_data[12] << 8) | reg_data[11];
    uint16_t par_p6_raw = (uint16_t)(reg_data[14] << 8) | reg_data[13];
    int8_t par_p7_raw = (int8_t)reg_data[15];
    int8_t par_p8_raw = (int8_t)reg_data[16];
    int16_t par_p9_raw = (int16_t)((reg_data[18] << 8) | reg_data[17]);
    int8_t par_p10_raw = (int8_t)reg_data[19];
    int8_t par_p11_raw = (int8_t)reg_data[20];

    calib.par_p1 = ((double)par_p1_raw - 16384.0) / pow(2, 20);
    calib.par_p2 = ((double)par_p2_raw - 16384.0) / pow(2, 29);
    calib.par_p3 = (double)par_p3_raw / pow(2, 32);
    calib.par_p4 = (double)par_p4_raw / pow(2, 37);
    calib.par_p5 = (double)par_p5_raw / pow(2, -3);  // = par_p5_raw * 8
    calib.par_p6 = (double)par_p6_raw / pow(2, 6);
    calib.par_p7 = (double)par_p7_raw / pow(2, 8);
    calib.par_p8 = (double)par_p8_raw / pow(2, 15);
    calib.par_p9 = (double)par_p9_raw / pow(2, 48);
    calib.par_p10 = (double)par_p10_raw / pow(2, 48);
    calib.par_p11 = (double)par_p11_raw / pow(2, 65);

    calib.t_lin = 0.0;
}

// Read calibration data
int read_calib_data(void) {
    uint8_t calib_data[BMP390_CALIB_DATA_LEN];

    printf("Reading calibration data...\n");

    if (i2c_read(BMP390_REG_CALIB_DATA, calib_data, BMP390_CALIB_DATA_LEN) < 0) {
        fprintf(stderr, "Failed to read calibration data\n");
        return -1;
    }

    parse_calib_data(calib_data);

    printf("Calibration data loaded\n");
    return 0;
}

// Compensate temperature (returns temperature in Celsius)
double compensate_temperature(uint32_t uncomp_temp) {
    double partial_data1;
    double partial_data2;

    partial_data1 = (double)uncomp_temp - calib.par_t1;
    partial_data2 = partial_data1 * calib.par_t2;

    // Store t_lin for pressure compensation
    calib.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib.par_t3;

    return calib.t_lin;
}

// Compensate pressure (returns pressure in Pa)
double compensate_pressure(uint32_t uncomp_press) {
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;

    // Calculate offset
    partial_data1 = calib.par_p6 * calib.t_lin;
    partial_data2 = calib.par_p7 * (calib.t_lin * calib.t_lin);
    partial_data3 = calib.par_p8 * (calib.t_lin * calib.t_lin * calib.t_lin);
    partial_out1 = calib.par_p5 + partial_data1 + partial_data2 + partial_data3;

    // Calculate sensitivity
    partial_data1 = calib.par_p2 * calib.t_lin;
    partial_data2 = calib.par_p3 * (calib.t_lin * calib.t_lin);
    partial_data3 = calib.par_p4 * (calib.t_lin * calib.t_lin * calib.t_lin);
    partial_out2 = (double)uncomp_press * (calib.par_p1 + partial_data1 + partial_data2 + partial_data3);

    // Calculate pressure
    partial_data1 = (double)uncomp_press * (double)uncomp_press;
    partial_data2 = calib.par_p9 + calib.par_p10 * calib.t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((double)uncomp_press * (double)uncomp_press * (double)uncomp_press) * calib.par_p11;

    return partial_out1 + partial_out2 + partial_data4;
}

// Calculate altitude from pressure using barometric formula
// Uses the global sea_level_pressure for reference
double calculate_altitude(double pressure) {
    // Hypsometric formula: altitude = 44330 * (1 - (P/P0)^(1/5.255))
    // P0 = sea level pressure (reference)
    return 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
}

// Calculate sea level pressure from known altitude and measured pressure
// Inverse of the hypsometric formula
double calculate_sea_level_pressure(double pressure, double altitude) {
    // P0 = P / (1 - altitude/44330)^5.255
    double ratio = 1.0 - (altitude / 44330.0);
    return pressure / pow(ratio, 5.255);
}

// Read and compensate sensor data
int read_sensor_data(double *pressure, double *temperature) {
    uint8_t data[6];

    // Check if data is ready
    uint8_t status;
    if (i2c_read(BMP390_REG_STATUS, &status, 1) < 0) {
        return -1;
    }

    // Read 6 bytes of data (pressure: 3 bytes + temperature: 3 bytes)
    if (i2c_read(BMP390_REG_DATA, data, 6) < 0) {
        return -1;
    }

    // Parse raw data (24-bit, little-endian)
    uint32_t uncomp_press = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
    uint32_t uncomp_temp = (uint32_t)data[3] | ((uint32_t)data[4] << 8) | ((uint32_t)data[5] << 16);

    // Compensate (temperature must be done first as it sets t_lin for pressure)
    *temperature = compensate_temperature(uncomp_temp);
    *pressure = compensate_pressure(uncomp_press);

    return 0;
}

// Calibrate using known altitude - average multiple readings for stability
void calibrate_altitude(void) {
    printf("Calibrating altitude reference...\n");
    printf("Known altitude: %.1f m\n", known_altitude);

    // Take multiple readings and average
    double pressure_sum = 0.0;
    int valid_readings = 0;
    const int num_readings = 10;

    for (int i = 0; i < num_readings; i++) {
        double pressure, temperature;
        if (read_sensor_data(&pressure, &temperature) == 0) {
            pressure_sum += pressure;
            valid_readings++;
        }
        delay_ms(100);
    }

    if (valid_readings > 0) {
        double avg_pressure = pressure_sum / valid_readings;
        sea_level_pressure = calculate_sea_level_pressure(avg_pressure, known_altitude);
        printf("Measured pressure: %.2f Pa (avg of %d readings)\n", avg_pressure, valid_readings);
        printf("Calculated QNH: %.2f hPa (%.2f Pa)\n", sea_level_pressure / 100.0, sea_level_pressure);
    } else {
        fprintf(stderr, "Warning: Could not read sensor for calibration, using default\n");
    }
}

// Initialize BMP390
int init_bmp390(void) {
    uint8_t chip_id;

    // Try primary I2C address
    printf("Testing I2C address 0x%02X\n", BMP390_I2C_ADDR_PRIMARY);

    if (i2c_init(I2C_DEVICE, BMP390_I2C_ADDR_PRIMARY) < 0) {
        // Try secondary address
        printf("Testing I2C address 0x%02X\n", BMP390_I2C_ADDR_SECONDARY);
        if (i2c_init(I2C_DEVICE, BMP390_I2C_ADDR_SECONDARY) < 0) {
            return 0;
        }
        bmp390_i2c_addr = BMP390_I2C_ADDR_SECONDARY;
    } else {
        bmp390_i2c_addr = BMP390_I2C_ADDR_PRIMARY;
    }

    // Read and verify chip ID
    if (i2c_read(BMP390_REG_CHIP_ID, &chip_id, 1) < 0) {
        fprintf(stderr, "Failed to read chip ID\n");
        // Try secondary address
        i2c_close();
        printf("Testing I2C address 0x%02X\n", BMP390_I2C_ADDR_SECONDARY);
        if (i2c_init(I2C_DEVICE, BMP390_I2C_ADDR_SECONDARY) < 0) {
            return 0;
        }
        bmp390_i2c_addr = BMP390_I2C_ADDR_SECONDARY;
        if (i2c_read(BMP390_REG_CHIP_ID, &chip_id, 1) < 0) {
            fprintf(stderr, "Failed to read chip ID at secondary address\n");
            return 0;
        }
    }

    // Check for valid chip ID (BMP388 or BMP390)
    if (chip_id != BMP390_CHIP_ID && chip_id != BMP388_CHIP_ID) {
        fprintf(stderr, "Invalid chip ID: 0x%02X (expected 0x%02X or 0x%02X)\n",
                chip_id, BMP390_CHIP_ID, BMP388_CHIP_ID);
        return 0;
    }

    chip_id_found = chip_id;
    const char *chip_name = (chip_id == BMP390_CHIP_ID) ? "BMP390" : "BMP388";
    printf("Found %s at address 0x%02X (Chip ID: 0x%02X)\n", chip_name, bmp390_i2c_addr, chip_id);

    // Perform soft reset
    printf("Performing soft reset...\n");
    i2c_write(BMP390_REG_CMD, BMP390_CMD_SOFTRESET);
    delay_ms(10);  // Wait for reset to complete

    // Re-read chip ID to verify reset succeeded
    if (i2c_read(BMP390_REG_CHIP_ID, &chip_id, 1) < 0) {
        fprintf(stderr, "Failed to read chip ID after reset\n");
        return 0;
    }

    // Read calibration data
    if (read_calib_data() < 0) {
        fprintf(stderr, "Failed to read calibration data\n");
        return 0;
    }

    // Configure oversampling: pressure x8, temperature x1
    printf("Configuring oversampling...\n");
    uint8_t osr = (BMP390_OSR_X8 << 0) | (BMP390_OSR_X1 << 3);  // osr_p[2:0], osr_t[5:3]
    if (i2c_write(BMP390_REG_OSR, osr) < 0) {
        return 0;
    }

    // Configure ODR: 50Hz
    printf("Configuring ODR...\n");
    if (i2c_write(BMP390_REG_ODR, BMP390_ODR_50_HZ) < 0) {
        return 0;
    }

    // Configure IIR filter: coefficient 3
    printf("Configuring IIR filter...\n");
    uint8_t config = 0x02 << 1;  // iir_filter[3:1] = 2 (coeff = 3)
    if (i2c_write(BMP390_REG_CONFIG, config) < 0) {
        return 0;
    }

    // Enable pressure and temperature measurement in normal mode
    printf("Enabling sensors in normal mode...\n");
    uint8_t pwr_ctrl = BMP390_PRESS_EN | BMP390_TEMP_EN | (BMP390_MODE_NORMAL << 4);
    if (i2c_write(BMP390_REG_PWR_CTRL, pwr_ctrl) < 0) {
        return 0;
    }
    delay_ms(50);  // Wait for first measurement

    // Verify settings
    uint8_t pwr_ctrl_read;
    if (i2c_read(BMP390_REG_PWR_CTRL, &pwr_ctrl_read, 1) < 0) {
        fprintf(stderr, "Failed to verify power control\n");
        return 0;
    }
    printf("Power Control: 0x%02X\n", pwr_ctrl_read);

    return 1;
}

int main(int argc, char *argv[]) {
    int opt;

    // Parse command line arguments
    while ((opt = getopt(argc, argv, "q:a:h")) != -1) {
        switch (opt) {
            case 'q':
                // QNH in hPa, convert to Pa
                sea_level_pressure = atof(optarg) * 100.0;
                if (sea_level_pressure < 80000.0 || sea_level_pressure > 120000.0) {
                    fprintf(stderr, "Warning: QNH %.2f hPa seems unusual (typical: 980-1050 hPa)\n",
                            sea_level_pressure / 100.0);
                }
                printf("Using QNH: %.2f hPa (%.2f Pa)\n", sea_level_pressure / 100.0, sea_level_pressure);
                break;

            case 'a':
                calibrate_with_altitude = 1;
                known_altitude = atof(optarg);
                printf("Will calibrate using known altitude: %.1f m\n", known_altitude);
                break;

            case 'h':
                print_usage(argv[0]);
                return 0;

            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    printf("BMP390 I2C Example for Raspberry Pi\n");

    delay_ms(200);  // Wait for sensor power-up

    if (!init_bmp390()) {
        fprintf(stderr, "BMP390 failed to initialize! Check wiring and power supply.\n");
        i2c_close();
        return 1;
    }

    // Perform altitude calibration if requested
    if (calibrate_with_altitude) {
        calibrate_altitude();
    }

    const char *chip_name = (chip_id_found == BMP390_CHIP_ID) ? "BMP390" : "BMP388";
    printf("%s ready\n", chip_name);
    printf("Reference pressure (QNH): %.2f hPa\n", sea_level_pressure / 100.0);
    printf("Format: pressure,temperature,altitude\n");
    printf("Units: Pa,C,m\n");

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

        double pressure, temperature;

        if (read_sensor_data(&pressure, &temperature) == 0) {
            double altitude = calculate_altitude(pressure);
            printf("%.2f,%.2f,%.2f\n", pressure, temperature, altitude);
            fflush(stdout);
        }
    }

    i2c_close();
    return 0;
}
