#ifndef IMU_READ_H
#define IMU_READ_H

#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#define SAMPLE_RATE 100 // Sample rate in Hz
#define SAMPLE_DURATION 5 // 5 seconds of data
#define SAMPLE_SIZE (SAMPLE_RATE * SAMPLE_DURATION) // Number of samples to read

#define NUMBER_OF_IMUS 6 // Number of IMUs

struct IMUData {
    uint8_t st1;
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t temp;
    int16_t magX, magY, magZ;
    uint8_t st2;
};

struct IMUDatalist {
    struct IMUData data[SAMPLE_SIZE];
    int data_index; // Index to keep track of the current data point
    const char* label; // Label for the IMU
    spi_device_handle_t spi; // SPI device handle for the IMU
};

extern struct IMUDatalist imu_data[NUMBER_OF_IMUS];

void read_imu_data(struct IMUDatalist * imu_data, int imu_index);
void print_imu_data(struct IMUDatalist * imu_data, int imu_index);
void print_imu_data_csv(struct IMUDatalist * imu_data, int imu_index);
//void read_fifo(spi_device_handle_t spi);

#endif // IMU_READ_H
