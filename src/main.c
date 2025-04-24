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
#include "gesturelibrary.h"

#include "espnow_comm.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

// Timer handles
esp_timer_handle_t data_timer;
TaskHandle_t imu_read_task_handle; // Task handle for IMU reading task

// Array to hold the CS pin numbers
const gpio_num_t cs_pins[] = {IMU_CS_BOH, IMU_CS_THUMB, IMU_CS_INDEX, IMU_CS_MIDDLE, IMU_CS_RING, IMU_CS_PINKY};

// SPI device handles for each IMU setup in main
spi_device_handle_t imu_handles[6];

// Semaphore for printing orientation data
SemaphoreHandle_t print_mutex;

// Data Queues
QueueHandle_t imuQueue; // holds an index of IMU data to be processed
QueueHandle_t gestureQueue; // holds pointers to GestureOrientationData structs
QueueHandle_t orientationQueue; // holds pointers to OrientationDatalist structs
QueueHandle_t rx_gesture_queue; // Queue to hold recieved gesture data

/*
Main function to initialize the application
The application initializes the IMUs, sets up the button, and creates tasks for reading IMU data and processing gestures.

The IMU reading task is blocked until it receives a notification from a timer, which is set up to trigger every 10ms.
The IMU data is then sent to a queue for processing by the orientation task
The orientation task processes the IMU data and updates the orientation data for each IMU
The orientation data is then sent to a queue for processing
The gesture task passes the orientation data to the gesture functions to process the gesture data
The gesture functions will then send completed gestures to Secondary module for command playback
*/
void app_main() {

    vTaskDelay(pdMS_TO_TICKS(10000)); // Delay to allow for system initialization
    
    init_espnow(); // Initialize ESP-NOW for communication

    print_mutex = xSemaphoreCreateMutex();
    imu_data_queue_init(); // Initialize the IMU data queue
    ESP_LOGI(TAG, "IMU data queue initialized");

    gesture_queue_init(); // Initialize the gesture data queue
    ESP_LOGI(TAG, "Gesture data queue initialized");

    //init_uart_for_input(); // Initialize UART for input
    //ESP_LOGI(TAG, "UART initialized for input");

    ESP_LOGI(TAG, "Initializing SPI");
    ESP_ERROR_CHECK(spi_init());

    ESP_LOGI(TAG, "Initializing IMUs");
    setup_imu(imu_handles, 0);
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
    xTaskCreatePinnedToCore(imu_orientation_worker_task, "Worker0", 6144, NULL, 4, NULL, 0);
    //xTaskCreatePinnedToCore(imu_orientation_worker_task, "Worker1", 6144, NULL, 4, NULL, 1);
    //xTaskCreatePinnedToCore(gesture_worker_task, "GestureTask", 6144, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(gesture_worker_task, "GestureTask", 6144, NULL, 4, NULL, 1);


    // Set up periodic data timer
    setup_data_timer();

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Main task does nothing, just keeps the app running
    }
    // Note: The FIFO reading is now handled by the timer callback
}

