#include <stdio.h>
#include <stdlib.h> // Required for rand()
#include <stdint.h> // Required for int16_t, uint32_t, int32_t
#include <math.h>
#include <stdbool.h>
#include "bmi08x.h" // Includes bmi08.h under the hood
#include "coines.h"  // COINES platform SDK header

// ------------------ Engineering Configurations ------------------
#define CALIBRATION_SAMPLES   100     // Number of samples (approx. 1 second at 100Hz)
#define CAL_SAMPLE_DELAY_MS   10      // 10ms sampling interval
#define STATIONARY_THRESHOLD  25.0f   // Max peak-to-peak noise allowed during static state

// Config settings assumed:
// Gyro Range: 2000 dps  -> 16.384 LSB/dps
// Accel Range: 3g       -> 10920 LSB/g
#define ACCEL_LSB_PER_G       10920.0f
#define GYRO_LSB_PER_DPS      16.384f

// ------------------ Data Structures ------------------
typedef struct {
    float ax_offset;
    float ay_offset;
    float az_offset; 
    
    float gx_offset;
    float gy_offset;
    float gz_offset;
    
    float cal_temperature; 
} imu_calibration_data_t;

typedef struct {
    float gx_temp_coeff; 
    float gy_temp_coeff;
    float gz_temp_coeff;
} imu_temp_compensation_t;

// Global Calibration Variables
imu_calibration_data_t cal_data = {0};
imu_temp_compensation_t temp_coeff = {
    .gx_temp_coeff = 0.05f,   
    .gy_temp_coeff = -0.03f,
    .gz_temp_coeff = 0.04f
};
bool is_calibrated = false;

// ------------------ Helper Temperature Function ------------------
/**
 * @brief Helper to read and convert raw temperature to Celsius
 */
float read_temperature_c(struct bmi08_dev *dev)
{
    int32_t raw_temp = 0;
    int8_t rslt = bmi08a_get_sensor_temperature(dev, &raw_temp);
    if (rslt == BMI08_OK) {
        // Temperature formula from BMI088 datasheet: (raw_temp * 0.125) + 23.0
        return ((float)raw_temp * 0.125f) + 23.0f;
    }
    return 0.0f;
}

// ------------------ Algorithms ------------------

/**
 * @brief Checks if the IMU is completely stationary.
 */
bool is_imu_stationary(struct bmi08_dev *dev)
{
    struct bmi08_sensor_data temp_gyro;
    int32_t min_x = 32767, max_x = -32768;
    int32_t min_y = 32767, max_y = -32768;
    int32_t min_z = 32767, max_z = -32768;

    for (int i = 0; i < 50; i++) {
        if (bmi08g_get_data(&temp_gyro, dev) == BMI08_OK) {
            if (temp_gyro.x < min_x) min_x = temp_gyro.x;
            if (temp_gyro.x > max_x) max_x = temp_gyro.x;
            if (temp_gyro.y < min_y) min_y = temp_gyro.y;
            if (temp_gyro.y > max_y) max_y = temp_gyro.y;
            if (temp_gyro.z < min_z) min_z = temp_gyro.z;
            if (temp_gyro.z > max_z) max_z = temp_gyro.z;
        }
        coines_delay_msec(5); 
    }

    if ((max_x - min_x) < STATIONARY_THRESHOLD &&
        (max_y - min_y) < STATIONARY_THRESHOLD &&
        (max_z - min_z) < STATIONARY_THRESHOLD) {
        return true;
    }
    return false;
}

/**
 * @brief Performs IMU Offset Calibration.
 */
bool perform_imu_calibration(struct bmi08_dev *dev)
{
    printf("[CALIBRATION] Detecting stationary state...\n");
    if (!is_imu_stationary(dev)) {
        printf("[CALIBRATION] ERROR: Motion detected! Place the device still.\n");
        return false;
    }

    printf("[CALIBRATION] Calibrating. Please do not move the device...\n");
    
    double ax_sum = 0, ay_sum = 0, az_sum = 0;
    double gx_sum = 0, gy_sum = 0, gz_sum = 0;
    double temp_sum = 0;
    
    struct bmi08_sensor_data accel_raw;
    struct bmi08_sensor_data gyro_raw;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        bmi08a_get_data(&accel_raw, dev);
        bmi08g_get_data(&gyro_raw, dev);
        
        float current_temp_c = read_temperature_c(dev);

        ax_sum += accel_raw.x;
        ay_sum += accel_raw.y;
        az_sum += accel_raw.z;
        
        gx_sum += gyro_raw.x;
        gy_sum += gyro_raw.y;
        gz_sum += gyro_raw.z;
        
        temp_sum += current_temp_c;

        coines_delay_msec(CAL_SAMPLE_DELAY_MS);
    }

    cal_data.ax_offset = (float)(ax_sum / CALIBRATION_SAMPLES);
    cal_data.ay_offset = (float)(ay_sum / CALIBRATION_SAMPLES);
    cal_data.az_offset = (float)(az_sum / CALIBRATION_SAMPLES) - ACCEL_LSB_PER_G;

    cal_data.gx_offset = (float)(gx_sum / CALIBRATION_SAMPLES);
    cal_data.gy_offset = (float)(gy_sum / CALIBRATION_SAMPLES);
    cal_data.gz_offset = (float)(gz_sum / CALIBRATION_SAMPLES);
    
    cal_data.cal_temperature = (float)(temp_sum / CALIBRATION_SAMPLES);

    is_calibrated = true;
    
    printf("[CALIBRATION] SUCCESS!\n");
    printf(" >> Baseline Temp: %.2f *C\n", cal_data.cal_temperature);
    printf(" >> Acc Offsets  -> X: %.1f | Y: %.1f | Z: %.1f (LSB)\n", cal_data.ax_offset, cal_data.ay_offset, cal_data.az_offset);
    printf(" >> Gyro Offsets -> X: %.1f | Y: %.1f | Z: %.1f (LSB)\n", cal_data.gx_offset, cal_data.gy_offset, cal_data.gz_offset);
    printf("-----------------------------------------------------------------------\n");

    return true;
}

/**
 * @brief Reads real-time IMU data with Bias and Temperature Compensation applied.
 */
void get_compensated_imu_data(struct bmi08_dev *dev, float *comp_accel, float *comp_gyro)
{
    struct bmi08_sensor_data raw_acc;
    struct bmi08_sensor_data raw_gyr;
    float current_temp = read_temperature_c(dev);

    bmi08a_get_data(&raw_acc, dev);
    bmi08g_get_data(&raw_gyr, dev);

    if (!is_calibrated) {
        comp_accel[0] = (float)raw_acc.x / ACCEL_LSB_PER_G;
        comp_accel[1] = (float)raw_acc.y / ACCEL_LSB_PER_G;
        comp_accel[2] = (float)raw_acc.z / ACCEL_LSB_PER_G;

        comp_gyro[0] = (float)raw_gyr.x / GYRO_LSB_PER_DPS;
        comp_gyro[1] = (float)raw_gyr.y / GYRO_LSB_PER_DPS;
        comp_gyro[2] = (float)raw_gyr.z / GYRO_LSB_PER_DPS;
        return;
    }

    float delta_temp = current_temp - cal_data.cal_temperature;

    float gx_temp_drift = delta_temp * temp_coeff.gx_temp_coeff;
    float gy_temp_drift = delta_temp * temp_coeff.gy_temp_coeff;
    float gz_temp_drift = delta_temp * temp_coeff.gz_temp_coeff;

    float corrected_ax = (float)raw_acc.x - cal_data.ax_offset;
    float corrected_ay = (float)raw_acc.y - cal_data.ay_offset;
    float corrected_az = (float)raw_acc.z - cal_data.az_offset;

    float corrected_gx = (float)raw_gyr.x - cal_data.gx_offset - gx_temp_drift;
    float corrected_gy = (float)raw_gyr.y - cal_data.gy_offset - gy_temp_drift;
    float corrected_gz = (float)raw_gyr.z - cal_data.gz_offset - gz_temp_drift;

    comp_accel[0] = corrected_ax / ACCEL_LSB_PER_G;
    comp_accel[1] = corrected_ay / ACCEL_LSB_PER_G;
    comp_accel[2] = corrected_az / ACCEL_LSB_PER_G;

    comp_gyro[0] = corrected_gx / GYRO_LSB_PER_DPS;
    comp_gyro[1] = corrected_gy / GYRO_LSB_PER_DPS;
    comp_gyro[2] = corrected_gz / GYRO_LSB_PER_DPS;
}

// ------------------ MAIN FUNCTION ------------------

int main(int argc, char *argv[])
{
    struct bmi08_dev bmi088_device;
    int8_t rslt;
    float accel_p[3] = {0}; 
    float gyro_p[3] = {0};  

    // 1. Initialize COINES hardware abstraction layer
    rslt = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);
    if (rslt != COINES_SUCCESS) {
        printf("COINES Communication Initialization Failed! Error: %d\n", rslt);
        return -1;
    }

    // 2. Map SPI or I2C functions using the driver's interface structure
    // (In actual execution, this matches the hardware setup steps shown in common_ports.c)
    bmi088_device.intf = BMI08_I2C_INTF; // Or BMI08_SPI_INTF depending on hardware selection
    
    // Setting up driver pointers matching the structure signature inside bmi08.h
    rslt = bmi08g_init(&bmi088_device);
    if (rslt != BMI08_OK) {
        printf("IMU Initialization Failed with Code: %d\n", rslt);
    }

    printf("==================== BMI088 PC COINES EXPERIMENT ====================\n\n");

    printf("[RUN] Starting live test loop...\n");
    
    // We execute the calibration
    perform_imu_calibration(&bmi088_device);

    for (int i = 0; i < 5; i++) {
        get_compensated_imu_data(&bmi088_device, accel_p, gyro_p);
        printf("[%d/5] Accel -> X: %+.3fg, Y: %+.3fg, Z: %+.3fg | Gyro -> X: %+.3fdps, Y: %+.3fdps\n", 
               i+1, accel_p[0], accel_p[1], accel_p[2], gyro_p[0], gyro_p[1]);
        coines_delay_msec(500);
    }

    // 3. Clean up and close connection
    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
    printf("\n==================== TEST ENDED ====================\n");
    return 0;
}
