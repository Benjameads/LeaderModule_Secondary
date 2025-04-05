// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "driver/spi_master.h"
// #include "esp_log.h"
// #include "esp_system.h"
// #include "esp_err.h"
// #include "nvs_flash.h"
// #include "esp_now.h"
// #include "esp_wifi.h"
// #include "esp_timer.h"

// // Logging tag for debugging
// #define TAG "IMU_SENSOR"

// // SPI configuration
// #define SPI_HOST SPI2_HOST
// #define DMA_CHAN 2  // DMA channel for SPI transactions

// // Chip Select (CS) pins for multiple IMUs
// #define IMU_CS_RING   GPIO_NUM_3
// #define IMU_CS_INDEX  GPIO_NUM_4  
// #define IMU_CS_MIDDLE GPIO_NUM_5
// #define IMU_CS_THUMB  GPIO_NUM_6
// #define IMU_CS_BOH    GPIO_NUM_43
// #define IMU_CS_PINKY  GPIO_NUM_44

// // SPI pin mappings
// #define MOSI GPIO_NUM_9
// #define MISO GPIO_NUM_8
// #define SCLK GPIO_NUM_7

// //MHz define
// #define MHz 1000000

// // Array to hold the CS pin numbers
// static const gpio_num_t cs_pins[] = {IMU_CS_RING, IMU_CS_INDEX, IMU_CS_MIDDLE, IMU_CS_THUMB, IMU_CS_BOH, IMU_CS_PINKY};
// const char* imu_labels[] = {"Ring", "Index", "Middle", "Thumb", "BOH", "Pinky"};

// //Timer handles
// esp_timer_handle_t data_timer;

// // SPI device handles for each IMU
// static spi_device_handle_t imu_handles[6];
// static TaskHandle_t imu_read_task_handle;

// Function Prototypes
// void read_imu_data_task(void* arg);
// esp_err_t spi_init();
// void setup_imu(spi_device_handle_t spi);
// void setup_fifo(spi_device_handle_t spi);
// void setup_magnetometer(spi_device_handle_t spi);
// void read_fifo(spi_device_handle_t spi);
// void data_timer_callback(void* arg);
// void setup_data_timer();
// void read_imu_data(spi_device_handle_t spi, int imu_index);
// uint8_t read_register(spi_device_handle_t spi, uint8_t reg);
// esp_err_t burst_read_registers(spi_device_handle_t spi, uint8_t start_reg, uint8_t* buffer, size_t length);
// void write_register(spi_device_handle_t spi, uint8_t reg, uint8_t value);
// void set_register_bits(spi_device_handle_t spi, uint8_t reg, uint8_t bits_to_set);
// void clear_register_bits(spi_device_handle_t spi, uint8_t reg, uint8_t bits_to_clear);
// void select_user_bank(spi_device_handle_t spi, uint8_t bank);

// void app_main() {
    
//     vTaskDelay(pdMS_TO_TICKS(10000)); //10 seconds delay before starting the timer

//     ESP_LOGI(TAG, "Initializing SPI");
//     ESP_ERROR_CHECK(spi_init());

//     ESP_LOGI(TAG, "Initializing IMUs");
//     for (int i = 0; i < 6; i++) {
//         setup_imu(imu_handles[i]);
//     }

//     // // Periodically read FIFO data from each IMU
//     // while (1) {
//     //     ESP_LOGI(TAG, "Reading FIFO Data");
//     //     for (int i = 0; i < 6; i++) {
//     //         read_fifo(imu_handles[i]);
//     //     }
//     //     vTaskDelay(pdMS_TO_TICKS(10)); // Delay to control read frequency
//     // }

//     xTaskCreatePinnedToCore(read_imu_data_task, "imu_read", 4096, NULL, 5, &imu_read_task_handle, 1); // Create task to read IMU data (will be notified/started by timer)

//     // Set up periodic data timer
//     setup_data_timer();

//     while(1) {
//         vTaskDelay(pdMS_TO_TICKS(1000)); // Main task does nothing, just keeps the app running
//     }
//     // Note: The FIFO reading is now handled by the timer callback
// }

// void read_imu_data_task(void* arg) {
//     while (1) {
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for notification
//         //ESP_LOGI(TAG, "=== IMU Task: Reading all IMUs ===");
//         for (uint8_t i = 0; i < 6; i++) {
//             //ESP_LOGI(TAG, "Reading IMU %d", i);
//             read_imu_data(imu_handles[i], i);
//         }
//         //ESP_LOGI(TAG, "=== IMU Task: Done reading all IMUs ===");
//     }
// }

// // Initialize SPI bus and configure each IMU as an SPI device
// esp_err_t spi_init() {
//     spi_bus_config_t buscfg = {
//         .mosi_io_num = MOSI,
//         .miso_io_num = MISO,
//         .sclk_io_num = SCLK,
//         .quadwp_io_num = -1,    // Not using Quad-SPI Write Protect
//         .quadhd_io_num = -1     // Not using Quad-SPI Hold
//     };
//     ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

//     // Initialize each IMU as an SPI device
//     for (int i = 0; i < 6; i++) {
//         spi_device_interface_config_t devcfg = {
//             .clock_speed_hz = 7*MHz,  // SPI clock speed set to 7 MHz
//             .mode = 0,                      // SPI Mode 0 (CPOL=0, CPHA=0) -> Clock idle LOW, data sampled on rising edge
//             .spics_io_num = cs_pins[i],     // Chip Select (CS) pin

//             .command_bits = 0,              // Number of bits in command phase (not used, so set to 0)
//             .address_bits = 0,              // Number of bits in address phase (not used, so set to 0)
//             .dummy_bits = 0,                // Number of dummy bits (for devices needing delay cycles, set to 0 here)
//             .duty_cycle_pos = 0,            // Unused (only relevant for special timing requirements)
//             .cs_ena_pretrans = 0,           // Delay (in SPI clock cycles) before activating CS (not used, set to 0)
//             .cs_ena_posttrans = 0,          // Delay (in SPI clock cycles) after deactivating CS (not used, set to 0)
//             .input_delay_ns = 0,            // No additional input delay (useful for compensating signal propagation delays)
//             .flags = 0,                     // No special SPI flags used (default behavior)
//             .queue_size = 1,                // Number of transactions that can be queued at a time (1 means no queueing)
//             .pre_cb = NULL,                 // No callback function before transmission
//             .post_cb = NULL                 // No callback function after transmission

//         };

//         ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &imu_handles[i]));
//     }
//     return ESP_OK;
// }

// // Function to set up the IMU with specific configurations
// //This function will set up accelerometr and gyroscope settings
// void setup_imu(spi_device_handle_t spi) {
//     uint8_t who_am_i = read_register(spi, 0x00);    // Read WHO_AM_I register
//     ESP_LOGI(TAG, "IMU WHO_AM_I: 0x%X", who_am_i);

//     if (who_am_i == 0xEA) { // Expected WHO_AM_I value
//         ESP_LOGI(TAG, "IMU Detected");
        
//         // Reset Device
//         set_register_bits(spi, 0x06, 0x80); // Set DEVICE_RESET bit in PWR_MGMT_1 register
//         while(read_register(spi, 0x06) & 0x80){ // Wait for DEVICE_RESET bit to clear
//             vTaskDelay(pdMS_TO_TICKS(1));
//         }


//         //Wake Device
//         while(!(read_register(spi, 0x06) & 0x40)){ // Wait for SLEEP bit to turn on
//             vTaskDelay(pdMS_TO_TICKS(1));
//         }
//         clear_register_bits(spi, 0x06, 0x40); // Clear SLEEP bit (bit 6)
//         set_register_bits(spi, 0x06, 0x01);   // Set clock to auto-select value 1-5

//         // Enable Gyroscope and Accelerometer
//         clear_register_bits(spi, 0x07, 0x3F); // Clear DISABLE_ACCEL and DISABLE_GYRO


//         // Switch to USER_BANK_2 for gyro/accel settings
//         select_user_bank(spi, 2);

//         // Set Accelerometer Scale to ±8g
//         clear_register_bits(spi, 0x14, 0x06);   // Clear both FS_SEL (bits 2:1)
//         set_register_bits(spi, 0x14, 0x05);     // Set FS_SEL = 0b10 and enable DLPF (bit 0)


//         // Set Gyroscope Scale to ±2000 dps
//         set_register_bits(spi, 0x01, 0x07);  // Set GYRO_FS_SEL to 0b11 (±2000 dps) and fchoice to 1

//         // Set Gyro sample rate to 100Hz
//         clear_register_bits(spi, 0x00, 0xFF); // Clear GYRO_SMPLRT_DIV bits
//         set_register_bits(spi, 0x00, 0x0A); // GYRO_SMPLRT_DIV = 10 (100Hz)

//         // Set Accel sample rate to 100Hz
//         clear_register_bits(spi, 0x09, 0x07); // Clear ACCEL_SMPLRT_DIV upper bits
//         clear_register_bits(spi, 0x10, 0xFF); // Clear ACCEL_SMPLRT_DIV lower bits
//         set_register_bits(spi, 0x10, 0x0A); // ACCEL_SMPLRT_DIV = 10 (100Hz)

//         // Switch back to USER_BANK_0
//         select_user_bank(spi, 0);

//         ESP_LOGI(TAG, "Accelerometer and Gyroscope setup complete");

//         // Call setup_fifo and setup_magnetometer functions Optionally
//         //setup_fifo(spi); // Set up FIFO buffer NOT WORKING***
//         setup_magnetometer(spi); // Set up magnetometer settings
//     } 
//     else {
//         ESP_LOGE(TAG, "IMU Not Detected");
//     }
// }

// // Function to set up the FIFO buffer 
// //*************This function is set up incorrectly, it will not work properly **************//
// void setup_fifo(spi_device_handle_t spi) {
//     // Reset FIFO
//     write_register(spi, 0x68, 0x1F);  // Assert reset
//     write_register(spi, 0x68, 0x00);  // Deassert reset

//     // Enable FIFO Mode (0x69)
//     write_register(spi, 0x69, 0x00);  // Stream Mode (old data overwritten when full)

//     // Enable FIFO for Gyro, Accel, and Temp
//     write_register(spi, 0x67, 0b00011110);

//     // Enable FIFO for Magnetometer
//     write_register(spi, 0x66, 0b00000001);
// }

// // Function to set up the Magnetometer settings
// void setup_magnetometer(spi_device_handle_t spi) {
//     // Enable I2C Master Mode
//     set_register_bits(spi, 0x03, 0x30); //Enable I2C_MST_EN and I2C_IF_DIS

//     // Switch to USER_BANK_3
//     select_user_bank(spi, 3);

//     //If we weren't using the Accelerometer and Gyroscope, we would need to set the I2C_MST_ODR_CONFIG register for the magnetometer to 100Hz, 
//     //but we are using the gyro and accel so the I2C master uses the same sample rate as the gyro and accel (100Hz)

//     // Configure I2C Master Settings
//     clear_register_bits(spi, 0x01, 0x8F);   // Clear I2C_MST_CLK (bit 3:0) and disable multi-master mode (bit 7)
//     set_register_bits(spi, 0x01, 0x07);     // Set I2C Master Clock (400 kHz)

//     // Configure I2C Slave 0 for Magnetometer
//     clear_register_bits(spi, 0x03, 0xFF);  // Clear I2C_SLV0_ADDR and I2C_SLV0_RNW (set to write)
//     set_register_bits(spi, 0x03, 0x0C);  // Set Slave 0 Address to 0x0C (Magnetometer)
//     clear_register_bits(spi, 0x04, 0xFF);  // Clear I2C_SLV0_REG
//     set_register_bits(spi, 0x04, 0x31);  // Set which I2C slave Register to read/write to CNTL2 (0x31)
//     write_register(spi, 0x06, 0x08);  // Set Magnetometer Continuous Measurement Mode (using write register because this register is used for internal I2C communication)
//     set_register_bits(spi, 0x03, 0x80);  // Clear I2C_SLV0_RNW (set to read)
//     clear_register_bits(spi, 0x04, 0xFF);  // Clear I2C_SLV0_REG
//     set_register_bits(spi, 0x04, 0x11);  // Set which I2C slave Register to read/write to Magnetometer Data register 0x11
//     set_register_bits(spi, 0x05, 0x86);  // Enable Read from Slave 0 (bit 7) and bits 3:0 set number of bytes to read (6 bytes for magnetometer axis data)
//     // SLV0 Read will start at 0x11 (Magnetometer Data Register) and read 6 bytes (0x11-0x16)

//     //Magnetometer Data Register Description:
//     // 0x11: Axis X LSB
//     // 0x12: Axis X MSB
//     // 0x13: Axis Y LSB
//     // 0x14: Axis Y MSB
//     // 0x15: Axis Z LSB
//     // 0x16: Axis Z MSB
//     //These registers will be accessed internally by the I2C Master
//     //The data will be read from the magnetometer automatically when the I2C_SLV0_EN bit is set in the I2C_SLV0_CTRL register (user bank 3 register 0x05)
//     //The I2C master puts this data in the EXT_SENS_DATA registers in the order above starting at EXT_SENS_DATA_00 (user bank 0 register 0x3B)

//     // Switch back to USER_BANK_0
//     select_user_bank(spi, 0);
//     ESP_LOGI(TAG, "Magnetometer setup complete");
// }


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
//         int16_t accelX = (buffer[0] << 8) | buffer[1];
//         int16_t accelY = (buffer[2] << 8) | buffer[3];
//         int16_t accelZ = (buffer[4] << 8) | buffer[5];
//         int16_t gyroX = (buffer[6] << 8) | buffer[7];
//         int16_t gyroY = (buffer[8] << 8) | buffer[9];
//         int16_t gyroZ = (buffer[10] << 8) | buffer[11];
//         int16_t magX = (buffer[12] << 8) | buffer[13];
//         int16_t magY = (buffer[14] << 8) | buffer[15];
//         int16_t magZ = (buffer[16] << 8) | buffer[17];
//         //uint8_t st2 = buffer[18]; // Read ST2 but do not output

//         ESP_LOGI(TAG, "FIFO: Accel(X:%.3f, Y:%.3f, Z:%.3f) Gyro(X:%.3f, Y:%.3f, Z:%.3f) Mag(X:%.3f, Y:%.3f, Z:%.3f)", 
//                 accelX / 4096.0, accelY / 4096.0, accelZ / 4096.0, 
//                 gyroX / 16.4, gyroY / 16.4, gyroZ / 16.4,
//                 magX * 0.15, magY * 0.15, magZ * 0.15);
//     } else {
//         ESP_LOGW(TAG, "FIFO Empty or not enough data");
//     }
// }

// void read_imu_data(spi_device_handle_t spi, int imu_index) {
//     const int max_attempts = 100; // Timeout safety (adjust as needed)
//     int attempt = 0;
//     uint16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp, magX, magY, magZ;

//     // Wait for data-ready bit to be set
//     while (attempt++ < max_attempts) {
//         uint8_t status = read_register(spi, 0x1A); // INT_STATUS_1
//         if (status & 0x01) break; // Bit 0 set = data ready
//         vTaskDelay(pdMS_TO_TICKS(1)); // Wait 1 ms before checking again
//     }

//     if (attempt >= max_attempts) {
//         ESP_LOGW(TAG, "Timed out waiting for IMU data");
//         return;
//     }

//     // Read IMU data registers
//     // Note: Magnetometer data is in reverse order (LSB first)

//     // Read Accelerometer data direct from registers
//     // int16_t accelX = (read_register(spi, 0x2D) << 8) | read_register(spi, 0x2E);
//     // int16_t accelY = (read_register(spi, 0x2F) << 8) | read_register(spi, 0x30);
//     // int16_t accelZ = (read_register(spi, 0x31) << 8) | read_register(spi, 0x32);
//     // int16_t gyroX = (read_register(spi, 0x33) << 8) | read_register(spi, 0x34);
//     // int16_t gyroY = (read_register(spi, 0x35) << 8) | read_register(spi, 0x36);
//     // int16_t gyroZ = (read_register(spi, 0x37) << 8) | read_register(spi, 0x38);
//     // int16_t magX = (read_register(spi, 0x3C) << 8) | read_register(spi, 0x3B);
//     // int16_t magY = (read_register(spi, 0x3E) << 8) | read_register(spi, 0x3D);
//     // int16_t magZ = (read_register(spi, 0x40) << 8) | read_register(spi, 0x3F);

//     uint8_t burst_data[19];
//     if (burst_read_registers(spi, 0x2D, burst_data, sizeof(burst_data)) == ESP_OK) {
//         accelX = (burst_data[0] << 8) | burst_data[1];
//         accelY = (burst_data[2] << 8) | burst_data[3];
//         accelZ = (burst_data[4] << 8) | burst_data[5];
//         gyroX  = (burst_data[6] << 8) | burst_data[7];
//         gyroY  = (burst_data[8] << 8) | burst_data[9];
//         gyroZ  = (burst_data[10] << 8) | burst_data[11];
//         temp   = (burst_data[12] << 8) | burst_data[13];
//         magX   = (burst_data[15] << 8) | burst_data[14];
//         magY   = (burst_data[17] << 8) | burst_data[16];
//         magZ   = (burst_data[19] << 8) | burst_data[18];

//         // ESP_LOGI(TAG, "IMU [%s]: Accel(X:%.3f, Y:%.3f, Z:%.3f) Gyro(X:%.3f, Y:%.3f, Z:%.3f) Mag(X:%.3f, Y:%.3f, Z:%.3f)",
//         //             imu_labels[imu_index],
//         //             accelX / 4096.0, accelY / 4096.0, accelZ / 4096.0, 
//         //             gyroX / 16.4, gyroY / 16.4, gyroZ / 16.4,
//         //             magX * 0.15, magY * 0.15, magZ * 0.15);
    
//         ESP_LOGI(TAG, "IMU [%s]: Accel(X:%d, Y:%d, Z:%d) Gyro(X:%d, Y:%d, Z:%d) Mag(X:%d, Y:%d, Z:%d)",
//                 imu_labels[imu_index],
//                 accelX, accelY, accelZ, 
//                 gyroX, gyroY, gyroZ,
//                 magX, magY, magZ);
//     }
// }


// void data_timer_callback(void* arg) {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     vTaskNotifyGiveFromISR(imu_read_task_handle, &xHigherPriorityTaskWoken);
//     if (xHigherPriorityTaskWoken) {
//         portYIELD_FROM_ISR();
//     }
// }

// void setup_data_timer() {
//     const esp_timer_create_args_t timer_args = {
//         .callback = &data_timer_callback,
//         .arg = NULL,
//         .name = "data_timer"
//     };

//     // Create the periodic timer
//     ESP_ERROR_CHECK(esp_timer_create(&timer_args, &data_timer));

//     // Start the timer with a period of 10,000 microseconds (100Hz)
//     ESP_ERROR_CHECK(esp_timer_start_periodic(data_timer, 100000));
// }

// // Read a register from the IMU over SPI
// uint8_t read_register(spi_device_handle_t spi, uint8_t reg) {
//     uint8_t tx_data[2] = { reg | 0x80, 0 };          // Set MSB high for read operation
//     uint8_t rx_data[2];                              // Buffer for received data
//     spi_transaction_t t = {
//         .length = 16,                     // Transaction length (8 bits address + 8 bits data)
//         .tx_buffer = tx_data,
//         .rx_buffer = rx_data
//     };
//     spi_device_transmit(spi, &t);                   // Perform SPI transaction
//     return rx_data[1];                              // Return the received data byte
// }

// // Read multiple registers from the IMU over SPI
// esp_err_t burst_read_registers(spi_device_handle_t spi, uint8_t start_reg, uint8_t* buffer, size_t length) {
//     if (length == 0 || buffer == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     uint8_t tx = start_reg | 0x80; // Set read bit
//     spi_transaction_t t = {
//         .length = (length + 1) * 8,
//         .tx_buffer = &tx,
//         .rx_buffer = buffer
//     };

//     esp_err_t ret = spi_device_transmit(spi, &t);
//     if (ret != ESP_OK) return ret;

//     return ESP_OK;
// }

// // Write a value to an IMU register over SPI
// void write_register(spi_device_handle_t spi, uint8_t reg, uint8_t value) {
//     uint8_t tx_data[2] = { reg & 0x7F, value };     // Clear MSB for write operation
//     spi_transaction_t t = {
//         .length = 16, // 8 bits register + 8 bits data
//         .tx_buffer = tx_data
//     };
//     spi_device_transmit(spi, &t);
// }


// //This function will set specific bits in a register(it will not clear any bits)
// void set_register_bits(spi_device_handle_t spi, uint8_t reg, uint8_t bits_to_set) {
//     uint8_t val = read_register(spi, reg);
//     val |= bits_to_set; //mask bits to set
//     write_register(spi, reg, val);
// }

// //This function will clear specific bits in a register(it will not set any bits)
// void clear_register_bits(spi_device_handle_t spi, uint8_t reg, uint8_t bits_to_clear) {
//     uint8_t val = read_register(spi, reg);
//     val &= ~bits_to_clear; //mask bits to clear
//     write_register(spi, reg, val);
// }

// void select_user_bank(spi_device_handle_t spi, uint8_t bank) {
//     if (bank > 3) return; // Only 0–3 are valid

//     // Read current value to preserve reserved bits
//     uint8_t val = read_register(spi, 0x7F);

//     // Clear USER_BANK bits (bits 5:4)
//     val &= ~(0x30);

//     // Set desired bank (bank << 4 positions it into bits 5:4)
//     val |= (bank << 4);

//     write_register(spi, 0x7F, val);
// }


