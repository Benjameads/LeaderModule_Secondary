#ifndef IMU_SPI_H
#define IMU_SPI_H

#include "driver/spi_master.h"
#include "esp_err.h"

// Logging tag for debugging
#define TAG "IMU_SENSOR"

// SPI configuration
#define SPI_HOST SPI2_HOST
#define DMA_CHAN 2  // DMA channel for SPI transactions

//MHz define
#define MHz 1000000

// SPI pin mappings
#define MOSI GPIO_NUM_9
#define MISO GPIO_NUM_8
#define SCLK GPIO_NUM_7

// Chip Select (CS) pins for multiple IMUs
#define IMU_CS_RING   GPIO_NUM_3
#define IMU_CS_INDEX  GPIO_NUM_4
#define IMU_CS_MIDDLE GPIO_NUM_5
#define IMU_CS_THUMB  GPIO_NUM_6
#define IMU_CS_BOH    GPIO_NUM_43
#define IMU_CS_PINKY  GPIO_NUM_44

extern const gpio_num_t cs_pins[6];
extern spi_device_handle_t imu_handles[6];

esp_err_t spi_init();
uint8_t read_register(spi_device_handle_t spi, uint8_t reg);
void write_register(spi_device_handle_t spi, uint8_t reg, uint8_t value);
void set_register_bits(spi_device_handle_t spi, uint8_t reg, uint8_t bits_to_set);
void clear_register_bits(spi_device_handle_t spi, uint8_t reg, uint8_t bits_to_clear);
void select_user_bank(spi_device_handle_t spi, uint8_t bank);
esp_err_t burst_read_registers(spi_device_handle_t spi, uint8_t start_reg, uint8_t* buffer, size_t length);

#endif // IMU_SPI_H
