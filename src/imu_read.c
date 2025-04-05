#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "imu_read.h"
#include "imu_spi.h"
#include "esp_task_wdt.h"

struct IMUDatalist imu_data[NUMBER_OF_IMUS]; // Array to hold IMU data for each IMU

void read_imu_data(struct IMUDatalist * imu_data, int imu_index) {
    const int max_attempts = 100; // Timeout safety (adjust as needed)
    int attempt = 0;
    uint16_t index; // Index for circular buffer
    //uint16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp, magX, magY, magZ;

    index = imu_data[imu_index].data_index; // Get the current index for the IMU data

    // Wait for data-ready bit to be set
    while (attempt++ < max_attempts) {
        uint8_t status = read_register(imu_data[imu_index].spi, 0x1A); // INT_STATUS_1
        if (status & 0x01) break; // Bit 0 set = data ready
        vTaskDelay(pdMS_TO_TICKS(1)); // Wait 1 ms before checking again
    }

    if (attempt >= max_attempts) {
        ESP_LOGW(TAG, "Timed out waiting for IMU data");
        return;
    }

    // Read IMU data registers
    // Note: Magnetometer data is in reverse order (LSB first)

    // Read Accelerometer data direct from registers
    // int16_t accelX = (read_register(spi, 0x2D) << 8) | read_register(spi, 0x2E);
    // int16_t accelY = (read_register(spi, 0x2F) << 8) | read_register(spi, 0x30);
    // int16_t accelZ = (read_register(spi, 0x31) << 8) | read_register(spi, 0x32);
    // int16_t gyroX = (read_register(spi, 0x33) << 8) | read_register(spi, 0x34);
    // int16_t gyroY = (read_register(spi, 0x35) << 8) | read_register(spi, 0x36);
    // int16_t gyroZ = (read_register(spi, 0x37) << 8) | read_register(spi, 0x38);
    // int16_t magX = (read_register(spi, 0x3C) << 8) | read_register(spi, 0x3B);
    // int16_t magY = (read_register(spi, 0x3E) << 8) | read_register(spi, 0x3D);
    // int16_t magZ = (read_register(spi, 0x40) << 8) | read_register(spi, 0x3F);

    //Read Data directly from registers
    // imu_data[imu_index].data[index].accelX = (read_register(imu_data[imu_index].spi, 0x2D) << 8) | read_register(imu_data[imu_index].spi, 0x2E);
    // imu_data[imu_index].data[index].accelY = (read_register(imu_data[imu_index].spi, 0x2F) << 8) | read_register(imu_data[imu_index].spi, 0x30);
    // imu_data[imu_index].data[index].accelZ = (read_register(imu_data[imu_index].spi, 0x31) << 8) | read_register(imu_data[imu_index].spi, 0x32);
    // imu_data[imu_index].data[index].gyroX  = (read_register(imu_data[imu_index].spi, 0x33) << 8) | read_register(imu_data[imu_index].spi, 0x34);
    // imu_data[imu_index].data[index].gyroY  = (read_register(imu_data[imu_index].spi, 0x35) << 8) | read_register(imu_data[imu_index].spi, 0x36);
    // imu_data[imu_index].data[index].gyroZ  = (read_register(imu_data[imu_index].spi, 0x37) << 8) | read_register(imu_data[imu_index].spi, 0x38);
    // //imu_data[imu_index].data[index].temp   = (read_register(imu_data[imu_index].spi, 0x39) << 8) | read_register(imu_data[imu_index].spi, 0x3A);
    // imu_data[imu_index].data[index].magX   = (read_register(imu_data[imu_index].spi, 0x3C) << 8) | read_register(imu_data[imu_index].spi, 0x3B);
    // imu_data[imu_index].data[index].magY   = (read_register(imu_data[imu_index].spi, 0x3E) << 8) | read_register(imu_data[imu_index].spi, 0x3D);
    // imu_data[imu_index].data[index].magZ   = (read_register(imu_data[imu_index].spi, 0x40) << 8) | read_register(imu_data[imu_index].spi, 0x3F);

    // ESP_LOGI(TAG, "IMU [%s]: Accel(X:%.3f, Y:%.3f, Z:%.3f) Gyro(X:%.3f, Y:%.3f, Z:%.3f) Mag(X:%.3f, Y:%.3f, Z:%.3f)",
    //             imu_data[imu_index].label,
    //             imu_data[imu_index].data[index].accelX / 4096.0, imu_data[imu_index].data[index].accelY / 4096.0, imu_data[imu_index].data[index].accelZ / 4096.0, 
    //             imu_data[imu_index].data[index].gyroX / 16.4, imu_data[imu_index].data[index].gyroY / 16.4, imu_data[imu_index].data[index].gyroZ / 16.4,
    //             imu_data[imu_index].data[index].magX * 0.15, imu_data[imu_index].data[index].magY * 0.15, imu_data[imu_index].data[index].magZ * 0.15);

    // // Increment data index and wrap around if necessary
    // imu_data[imu_index].data_index = (imu_data[imu_index].data_index + 1) % SAMPLE_SIZE; // Circular buffer index

    uint8_t burst_data[23];
    if (burst_read_registers(imu_data[imu_index].spi, 0x2D, burst_data, sizeof(burst_data)) == ESP_OK) {
        imu_data[imu_index].data[index].accelX = (burst_data[0] << 8) | burst_data[1];
        imu_data[imu_index].data[index].accelY = (burst_data[2] << 8) | burst_data[3];
        imu_data[imu_index].data[index].accelZ = (burst_data[4] << 8) | burst_data[5];
        imu_data[imu_index].data[index].gyroX  = (burst_data[6] << 8) | burst_data[7];
        imu_data[imu_index].data[index].gyroY  = (burst_data[8] << 8) | burst_data[9];
        imu_data[imu_index].data[index].gyroZ  = (burst_data[10] << 8) | burst_data[11];
        imu_data[imu_index].data[index].temp   = (burst_data[12] << 8) | burst_data[13];
        imu_data[imu_index].data[index].st1    = burst_data[14]; // Read ST1 register
        imu_data[imu_index].data[index].magX   = (burst_data[16] << 8) | burst_data[15];
        imu_data[imu_index].data[index].magY   = (burst_data[18] << 8) | burst_data[17];
        imu_data[imu_index].data[index].magZ   = (burst_data[20] << 8) | burst_data[19];
        imu_data[imu_index].data[index].st2    = burst_data[22]; // Read ST2 register

        // Increment data index and wrap around if necessary
        imu_data[imu_index].data_index = (imu_data[imu_index].data_index + 1) % SAMPLE_SIZE; // Circular buffer index
    }
}

void print_imu_data(struct IMUDatalist * imu_data, int imu_index) {
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        ESP_LOGI(TAG, "IMU [%s]: Sample %d: Accel(X:%.3f, Y:%.3f, Z:%.3f) Gyro(X:%.3f, Y:%.3f, Z:%.3f) Mag(X:%.3f, Y:%.3f, Z:%.3f) ST1:0x%02X ST2: 0x%02X",
                imu_data[imu_index].label, i,
                imu_data[imu_index].data[i].accelX / 4096.0, imu_data[imu_index].data[i].accelY / 4096.0, imu_data[imu_index].data[i].accelZ / 4096.0, 
                imu_data[imu_index].data[i].gyroX / 16.4, imu_data[imu_index].data[i].gyroY / 16.4, imu_data[imu_index].data[i].gyroZ / 16.4,
                imu_data[imu_index].data[i].magX * 0.15, imu_data[imu_index].data[i].magY * 0.15, imu_data[imu_index].data[i].magZ * 0.15, 
                imu_data[imu_index].data[i].st1, imu_data[imu_index].data[i].st2);

        if (i % 50 == 0) {
            esp_task_wdt_reset();  // Feed the watchdog every 50 samples
            vTaskDelay(pdMS_TO_TICKS(20));            // Let other tasks (like IDLE) run
        }
    }
}

void print_imu_data_csv(struct IMUDatalist * imu_data, int imu_index) {
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        float timestamp = i * 0.01f;  // 100 Hz = 10ms per sample

        printf("%d,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%02X,%02X\n",
            imu_index + 1, timestamp,
            imu_data[imu_index].data[i].accelX / 4096.0f,
            imu_data[imu_index].data[i].accelY / 4096.0f,
            imu_data[imu_index].data[i].accelZ / 4096.0f,
            imu_data[imu_index].data[i].gyroX / 16.4f,
            imu_data[imu_index].data[i].gyroY / 16.4f,
            imu_data[imu_index].data[i].gyroZ / 16.4f,
            imu_data[imu_index].data[i].magX * 0.15f,
            imu_data[imu_index].data[i].magY * 0.15f,
            imu_data[imu_index].data[i].magZ * 0.15f,
            imu_data[imu_index].data[i].st1,
            imu_data[imu_index].data[i].st2
        );

        if (i % 50 == 0) {
            esp_task_wdt_reset();  // Feed the watchdog every 50 samples
            vTaskDelay(pdMS_TO_TICKS(20));  // Let other tasks run
        }
    }
}


// // Read data from the FIFO buffer of an IMU
// void read_fifo(spi_device_handle_t spi) {
//     // Read FIFO count
//     uint8_t high_byte = read_register(spi, 0x70);
//     uint8_t low_byte = read_register(spi, 0x71);
//     int fifo_count = (high_byte << 8) | low_byte; // Combine high and low bytes

//     if (fifo_count >= 19) { // Ensure there's enough data for all 9-axis + ST2
//         uint8_t buffer[19]; // Buffer to store FIFO data
//         spi_transaction_t t = {
//             .length = 8 * 19, // Read 19 bytes (accel + gyro + magnetometer data + ST2)
//             .tx_buffer = NULL,
//             .rx_buffer = buffer
//         };
//         spi_device_transmit(spi, &t); // Perform SPI read

//         // Extract accelerometer, gyroscope, and magnetometer data
//         int16_t accelX = (buffer[0] << 8)  | buffer[1];
//         int16_t accelY = (buffer[2] << 8)  | buffer[3];
//         int16_t accelZ = (buffer[4] << 8)  | buffer[5];
//         int16_t gyroX  = (buffer[6] << 8)  | buffer[7];
//         int16_t gyroY  = (buffer[8] << 8)  | buffer[9];
//         int16_t gyroZ  = (buffer[10] << 8) | buffer[11];
//         int16_t magX   = (buffer[12] << 8) | buffer[13];
//         int16_t magY   = (buffer[14] << 8) | buffer[15];
//         int16_t magZ   = (buffer[16] << 8) | buffer[17];
//         //uint8_t st2 = buffer[18]; // Read ST2 but do not output

//         ESP_LOGI(TAG, "FIFO: Accel(X:%.3f, Y:%.3f, Z:%.3f) Gyro(X:%.3f, Y:%.3f, Z:%.3f) Mag(X:%.3f, Y:%.3f, Z:%.3f)", 
//                 accelX / 4096.0, accelY / 4096.0, accelZ / 4096.0, 
//                 gyroX / 16.4, gyroY / 16.4, gyroZ / 16.4,
//                 magX * 0.15, magY * 0.15, magZ * 0.15);
//     } else {
//         ESP_LOGW(TAG, "FIFO Empty or not enough data");
//     }
// }