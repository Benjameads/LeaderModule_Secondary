#ifndef IMU_SETUP_H
#define IMU_SETUP_H

#include "driver/spi_master.h"
#include "esp_err.h"

extern const char* imu_labels[]; // Labels for IMUs

#define ACCEL_FS_SEL_BITS (0x03 << 1) // Accelerometer full scale selection bits
#define ACCEL_FS_SEL_2G   (0x00 << 1) // ±2g
#define ACCEL_FS_SEL_4G   (0x01 << 1) // ±4g
#define ACCEL_FS_SEL_8G   (0x02 << 1) // ±8g
#define ACCEL_FS_SEL_16G  (0x03 << 1) // ±16g

#define GYRO_FS_SEL_BITS (0x03 << 1) // Gyroscope full scale selection bits
#define GYRO_FS_SEL_250  (0x00 << 1) // ±250 dps
#define GYRO_FS_SEL_500  (0x01 << 1) // ±500 dps
#define GYRO_FS_SEL_1000 (0x02 << 1) // ±1000 dps
#define GYRO_FS_SEL_2000 (0x03 << 1) // ±2000 dps

void setup_imu(spi_device_handle_t * spi, int i);
void setup_fifo(spi_device_handle_t spi);
void setup_magnetometer(spi_device_handle_t spi);

#endif // IMU_SETUP_H
