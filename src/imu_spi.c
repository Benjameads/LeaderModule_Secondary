#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "imu_spi.h"
#include <string.h>

// Initialize SPI bus and configure each IMU as an SPI device
esp_err_t spi_init() {
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = SCLK,
        .quadwp_io_num = -1,    // Not using Quad-SPI Write Protect
        .quadhd_io_num = -1     // Not using Quad-SPI Hold
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Initialize each IMU as an SPI device
    for (int i = 0; i < 6; i++) {
        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = 7*MHz,  // SPI clock speed set to 7 MHz
            .mode = 0,                      // SPI Mode 0 (CPOL=0, CPHA=0) -> Clock idle LOW, data sampled on rising edge
            .spics_io_num = cs_pins[i],     // Chip Select (CS) pin

            .command_bits = 0,              // Number of bits in command phase (not used, so set to 0)
            .address_bits = 0,              // Number of bits in address phase (not used, so set to 0)
            .dummy_bits = 0,                // Number of dummy bits (for devices needing delay cycles, set to 0 here)
            .duty_cycle_pos = 0,            // Unused (only relevant for special timing requirements)
            .cs_ena_pretrans = 0,           // Delay (in SPI clock cycles) before activating CS (not used, set to 0)
            .cs_ena_posttrans = 0,          // Delay (in SPI clock cycles) after deactivating CS (not used, set to 0)
            .input_delay_ns = 0,            // No additional input delay (useful for compensating signal propagation delays)
            .flags = 0,                     // No special SPI flags used (default behavior)
            .queue_size = 1,                // Number of transactions that can be queued at a time (1 means no queueing)
            .pre_cb = NULL,                 // No callback function before transmission
            .post_cb = NULL                 // No callback function after transmission

        };

        ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &imu_handles[i]));
    }
    return ESP_OK;
}

// Read a register from the IMU over SPI
uint8_t read_register(spi_device_handle_t spi, uint8_t reg) {
    uint8_t tx_data[2] = { reg | 0x80, 0 };          // Set MSB high for read operation
    uint8_t rx_data[2];                              // Buffer for received data
    spi_transaction_t t = {
        .length = 16,                     // Transaction length (8 bits address + 8 bits data)
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };
    spi_device_transmit(spi, &t);                   // Perform SPI transaction
    return rx_data[1];                              // Return the received data byte
}

// Write a value to an IMU register over SPI
void write_register(spi_device_handle_t spi, uint8_t reg, uint8_t value) {
    uint8_t tx_data[2] = { reg & 0x7F, value };     // Clear MSB for write operation
    spi_transaction_t t = {
        .length = 16, // 8 bits register + 8 bits data
        .tx_buffer = tx_data
    };
    spi_device_transmit(spi, &t);
}


//This function will set specific bits in a register(it will not clear any bits)
void set_register_bits(spi_device_handle_t spi, uint8_t reg, uint8_t bits_to_set) {
    uint8_t val = read_register(spi, reg);
    val |= bits_to_set; //mask bits to set
    write_register(spi, reg, val);
}

//This function will clear specific bits in a register(it will not set any bits)
void clear_register_bits(spi_device_handle_t spi, uint8_t reg, uint8_t bits_to_clear) {
    uint8_t val = read_register(spi, reg);
    val &= ~bits_to_clear; //mask bits to clear
    write_register(spi, reg, val);
}

void select_user_bank(spi_device_handle_t spi, uint8_t bank) {
    if (bank > 3) return; // Only 0–3 are valid

    // Read current value to preserve reserved bits
    uint8_t val = read_register(spi, 0x7F);

    // Clear USER_BANK bits (bits 5:4)
    val &= ~(0x30);

    // Set desired bank (bank << 4 positions it into bits 5:4)
    val |= (bank << 4);

    write_register(spi, 0x7F, val);
}


esp_err_t burst_read_registers(spi_device_handle_t spi, uint8_t start_reg, uint8_t* buffer, size_t length) {
    if (length == 0 || buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx_buffer[length + 1];
    uint8_t rx_buffer[length + 1];
    tx_buffer[0] = start_reg | 0x80;  // Set read bit

    // Fill the rest with dummy bytes
    memset(&tx_buffer[1], 0x00, length);

    spi_transaction_t t = {
        .length = (length + 1) * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer
    };

    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK) return ret;

    // Copy only the actual data (skip first byte, which is response to address byte)
    memcpy(buffer, &rx_buffer[1], length);

    return ESP_OK;
}
