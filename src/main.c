#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "imu_setup.h"
#include "imu_spi.h"
#include "data_timer.h"
#include "imu_read.h"
#include "MadgwickAHRS.h"
#include "imu_orientation.h"
#include "quaternion_utils.h"
#include "read_task.h"
#include "orientation_task.h"

#include "espnow_comm.h"
#include "esp_wifi.h"
#include "driver/uart.h"
#include "nvs_flash.h"

#define TEST_BYTE_TO_SEND 'A'


//Timer handles
esp_timer_handle_t data_timer;
TaskHandle_t imu_read_task_handle; // Task handle for IMU reading task

// Array to hold the CS pin numbers
const gpio_num_t cs_pins[] = {IMU_CS_BOH, IMU_CS_THUMB, IMU_CS_INDEX, IMU_CS_MIDDLE, IMU_CS_RING, IMU_CS_PINKY};

//SPI device handles for each IMU setup in main
spi_device_handle_t imu_handles[6];

// Semaphore for printing orientation data
SemaphoreHandle_t print_mutex;

// IMU Data Queue
QueueHandle_t imuQueue; // holds an index of IMU data to be processed

void app_main() {
    init_espnow();
    
    vTaskDelay(pdMS_TO_TICKS(10000)); //10 seconds delay before starting the timer

    print_mutex = xSemaphoreCreateMutex();
    imu_data_queue_init(); // Initialize the IMU data queue
    ESP_LOGI(TAG, "IMU data queue initialized");

    //init_uart_for_input(); // Initialize UART for input
    //ESP_LOGI(TAG, "UART initialized for input");

    ESP_LOGI(TAG, "Initializing SPI");
    ESP_ERROR_CHECK(spi_init());

    ESP_LOGI(TAG, "Initializing IMUs");
    for (int i = 0; i < 6; i++) {
        setup_imu(imu_handles, i); // Set up each IMU
    }

    setup_btn(); // Set up the button for user input

    gpio_config_t debugpin_conf = {
        .pin_bit_mask = 1ULL << DEBUGPIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&debugpin_conf); // Configure GPIO pin for output

    xTaskCreatePinnedToCore(read_imu_data_task, "imu_read", 6144, NULL, 5, &imu_read_task_handle, 1); // Create task to read IMU data (will be notified/started by timer)
    xTaskCreatePinnedToCore(imu_orientation_worker_task, "Worker0", 8192, NULL, 4, NULL, 0);
    //xTaskCreatePinnedToCore(imu_orientation_worker_task, "Worker1", 8192, NULL, 4, NULL, 1);


    // Set up periodic data timer
    setup_data_timer();

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Main task does nothing, just keeps the app running
    }
    // Note: The FIFO reading is now handled by the timer callback
}

