#ifndef IMU_READ_H
#define IMU_READ_H

#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#define SAMPLE_SIZE 1 // Number of raw data samples to store

#define NUMBER_OF_IMUS 6 // Number of IMUs

#define ACCEL_2G_SCALE (1.0f/16384.0f) // Scale factor for ±2g accelerometer data
#define ACCEL_4G_SCALE (1.0f/8192.0f) // Scale factor for ±4g accelerometer data
#define ACCEL_8G_SCALE (1.0f/4096.0f) // Scale factor for ±8g accelerometer data
#define ACCEL_16G_SCALE (1.0f/2048.0f) // Scale factor for ±16g accelerometer data

#define GYRO_250_SCALE (1.0f/131.0f) // Scale factor for ±250 dps gyroscope data
#define GYRO_500_SCALE (1.0f/65.5f) // Scale factor for ±500 dps gyroscope data
#define GYRO_1000_SCALE (1.0f/32.8f) // Scale factor for ±1000 dps gyroscope data
#define GYRO_2000_SCALE (1.0f/16.4f) // Scale factor for ±2000 dps gyroscope data

#define MAG_SCALE 0.15f // Scale factor for magnetometer data

struct IMUData {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t temp;
    uint8_t st1;
    int16_t magX, magY, magZ;
    uint8_t st2;
};

struct IMUDatalist {
    struct IMUData data[SAMPLE_SIZE];
    int data_index; // Index to keep track of the current data point
    uint8_t imu_index; // Index of the IMU in the array
    spi_device_handle_t spi; // SPI device handle for the IMU
};

typedef enum {
    BOH,
    THUMB,
    INDEX,
    MIDDLE,
    RING,
    PINKY
} IMU_INDEX;

//extern struct IMUDatalist imu_data[NUMBER_OF_IMUS];

void read_imu_data(struct IMUDatalist * imu_data, int imu_index);
void print_imu_data(struct IMUDatalist * imu_data, int imu_index);
void print_imu_data_csv(struct IMUDatalist * imu_data, int imu_index);
void read_fifo_data(struct IMUDatalist *imu_data, int imu_index);

#endif // IMU_READ_H
