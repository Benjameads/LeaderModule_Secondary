#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "imu_read.h"
#include "imu_spi.h"
#include "esp_task_wdt.h"
#include "imu_setup.h"

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

    uint8_t burst_data[23];
    if (burst_read_registers(imu_data[imu_index].spi, 0x2D, burst_data, sizeof(burst_data)) == ESP_OK) {
        imu_data[imu_index].data[index].accelX =  (burst_data[0] << 8) | burst_data[1];
        imu_data[imu_index].data[index].accelY =  (burst_data[2] << 8) | burst_data[3];
        imu_data[imu_index].data[index].accelZ =  (burst_data[4] << 8) | burst_data[5];
        imu_data[imu_index].data[index].gyroX  =  (burst_data[6] << 8) | burst_data[7];
        imu_data[imu_index].data[index].gyroY  =  (burst_data[8] << 8) | burst_data[9];
        imu_data[imu_index].data[index].gyroZ  =  (burst_data[10] << 8) | burst_data[11];
        imu_data[imu_index].data[index].temp   =  (burst_data[12] << 8) | burst_data[13];
        imu_data[imu_index].data[index].st1    =   burst_data[14]; // Read ST1 register
        imu_data[imu_index].data[index].magX   =  (burst_data[16] << 8) | burst_data[15];
        imu_data[imu_index].data[index].magY   =  (burst_data[18] << 8) | burst_data[17];
        imu_data[imu_index].data[index].magZ   =  (burst_data[20] << 8) | burst_data[19];
        imu_data[imu_index].data[index].st2    =   burst_data[22]; // Read ST2 register

        // Increment data index and wrap around if necessary
        imu_data[imu_index].data_index = (imu_data[imu_index].data_index + 1) % SAMPLE_SIZE; // Circular buffer index
    }
}

void print_imu_data(struct IMUDatalist * imu_data, int imu_index) {
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        ESP_LOGI(TAG, "IMU [%s]: Sample %d: Accel(X:%.3f, Y:%.3f, Z:%.3f) Gyro(X:%.3f, Y:%.3f, Z:%.3f) Mag(X:%.3f, Y:%.3f, Z:%.3f) ST1:0x%02X ST2: 0x%02X",
                imu_labels[imu_index], i,
                imu_data[imu_index].data[i].accelX * ACCEL_4G_SCALE, imu_data[imu_index].data[i].accelY * ACCEL_4G_SCALE, imu_data[imu_index].data[i].accelZ * ACCEL_4G_SCALE, 
                imu_data[imu_index].data[i].gyroX * GYRO_500_SCALE, imu_data[imu_index].data[i].gyroY * GYRO_500_SCALE, imu_data[imu_index].data[i].gyroZ * GYRO_500_SCALE,
                imu_data[imu_index].data[i].magX * MAG_SCALE, imu_data[imu_index].data[i].magY * MAG_SCALE, imu_data[imu_index].data[i].magZ * MAG_SCALE, 
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
            imu_data[imu_index].data[i].accelX * ACCEL_4G_SCALE,
            imu_data[imu_index].data[i].accelY * ACCEL_4G_SCALE,
            imu_data[imu_index].data[i].accelZ * ACCEL_4G_SCALE,
            imu_data[imu_index].data[i].gyroX * GYRO_500_SCALE,
            imu_data[imu_index].data[i].gyroY * GYRO_500_SCALE,
            imu_data[imu_index].data[i].gyroZ * GYRO_500_SCALE,
            imu_data[imu_index].data[i].magX * MAG_SCALE,
            imu_data[imu_index].data[i].magY * MAG_SCALE,
            imu_data[imu_index].data[i].magZ * MAG_SCALE,
            imu_data[imu_index].data[i].st1,
            imu_data[imu_index].data[i].st2
        );

        if (i % 50 == 0) {
            esp_task_wdt_reset();  // Feed the watchdog every 50 samples
            vTaskDelay(pdMS_TO_TICKS(20));  // Let other tasks run
        }
    }
}

void read_fifo_data(struct IMUDatalist *imu_data, int imu_index) {
    const int max_attempts = 100;  // Timeout safety (adjust as needed)
    int attempt = 0;
    uint16_t packet_size = 21;     // Expected FIFO packet size (6 accel + 6 gyro + 9 mag)
    uint16_t fifo_count = 0;

    // Poll FIFO count registers until we have at least one full packet
    while (attempt++ < max_attempts) {
        uint8_t count_h = read_register(imu_data[imu_index].spi, 0x70);  // FIFO_COUNTH
        uint8_t count_l = read_register(imu_data[imu_index].spi, 0x71);  // FIFO_COUNTL
        fifo_count = ((uint16_t)count_h << 8) | count_l;
        if (fifo_count >= packet_size) {
            break;  // Enough data available
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // Wait 1 ms before checking again
    }

    if (attempt >= max_attempts) {
        ESP_LOGW(TAG, "Timed out waiting for FIFO data");
        return;
    }

    // Read a complete packet from the FIFO. 
    // For temperature disabled, we expect 21 bytes:
    //   Bytes 0-5:  Accel (X, Y, Z; each 2 bytes)
    //   Bytes 6-11: Gyro (X, Y, Z; each 2 bytes)
    //   Bytes 12-20: Magnetometer packet:
    //                Byte 12: ST1
    //                Bytes 13-14: MagX (LSB first)
    //                Bytes 15-16: MagY (LSB first)
    //                Bytes 17-18: MagZ (LSB first)
    //                Byte 19: Dummy (discard)
    //                Byte 20: ST2
    uint8_t burst_data[packet_size];
    if (burst_read_registers(imu_data[imu_index].spi, 0x72, burst_data, packet_size) == ESP_OK) {
        // Get the current circular buffer index
        uint16_t index = imu_data[imu_index].data_index;

        // Parse accelerometer data
        imu_data[imu_index].data[index].accelX = (burst_data[0] << 8) | burst_data[1];
        imu_data[imu_index].data[index].accelY = (burst_data[2] << 8) | burst_data[3];
        imu_data[imu_index].data[index].accelZ = (burst_data[4] << 8) | burst_data[5];

        // Parse gyroscope data
        imu_data[imu_index].data[index].gyroX = (burst_data[6] << 8) | burst_data[7];
        imu_data[imu_index].data[index].gyroY = (burst_data[8] << 8) | burst_data[9];
        imu_data[imu_index].data[index].gyroZ = (burst_data[10] << 8) | burst_data[11];

        // Parse magnetometer data:
        // Note: The magnetometer returns its data with the LSB first.
        imu_data[imu_index].data[index].st1  = burst_data[12];  // Status 1
        imu_data[imu_index].data[index].magX = (burst_data[14] << 8) | burst_data[13];
        imu_data[imu_index].data[index].magY = (burst_data[16] << 8) | burst_data[15];
        imu_data[imu_index].data[index].magZ = (burst_data[18] << 8) | burst_data[17];
        imu_data[imu_index].data[index].st2  = burst_data[20];  // Status 2

        // Increment the circular buffer index (wrap around if needed)
        imu_data[imu_index].data_index = (index + 1) % SAMPLE_SIZE;
    }
}

