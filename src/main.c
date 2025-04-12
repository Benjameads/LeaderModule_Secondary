#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "imu_setup.h"
#include "imu_spi.h"
#include "data_timer.h"
#include "imu_read.h"
#include "uart.h"
#include "MadgwickAHRS.h"
#include "imu_orientation.h"
#include "quaternion_utils.h"
#include "read_task.h"
#include "orientation_task.h"

//Timer handles
esp_timer_handle_t data_timer;
TaskHandle_t imu_read_task_handle; // Task handle for IMU reading task
TaskHandle_t raw_to_orientation_task_handle; // Task handles for IMU reading and orientation tasks

// IMU Data Queue
QueueHandle_t imuQueue; // holds pointers to IMUDatalist structs

void app_main() {
    
    vTaskDelay(pdMS_TO_TICKS(10000)); //10 seconds delay before starting the timer

    //init_uart_for_input(); // Initialize UART for input
    //ESP_LOGI(TAG, "UART initialized for input");

    ESP_LOGI(TAG, "Initializing SPI");
    ESP_ERROR_CHECK(spi_init());

    ESP_LOGI(TAG, "Initializing IMUs");
    for (int i = 0; i < 6; i++) {
        setup_imu(imu_handles, i); // Set up each IMU
    }

    // Set up Data Structures for IMU data
    for (int i = 0; i < 6; i++) {
        imu_data[i].data_index = 0; // Initialize data index for each IMU
        imu_data[i].spi = imu_handles[i]; // Assign SPI handle to each IMU data structure
        imu_data[i].IMU_index = i; // Assign IMU index to each IMU data structure
    }

    // Configure the user button pin
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << USER_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf); // Configure the GPIO pin

    xTaskCreatePinnedToCore(read_imu_data_task, "imu_read", 6144, NULL, 5, &imu_read_task_handle, 1); // Create task to read IMU data (will be notified/started by timer)
    xTaskCreatePinnedToCore(imu_orientation_worker_task, "Worker0", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(imu_orientation_worker_task, "Worker1", 4096, NULL, 4, NULL, 1);


    // Set up periodic data timer
    setup_data_timer();

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Main task does nothing, just keeps the app running
    }
    // Note: The FIFO reading is now handled by the timer callback
}

