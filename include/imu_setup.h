#ifndef IMU_SETUP_H
#define IMU_SETUP_H

#include "driver/spi_master.h"
#include "esp_err.h"

void setup_imu(spi_device_handle_t spi);
void setup_fifo(spi_device_handle_t spi);
void setup_magnetometer(spi_device_handle_t spi);

#endif // IMU_SETUP_H
