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

#define USER_BUTTON_GPIO 0  // GPIO pin connected to the user button

//Function Prototypes
void read_imu_data_task(void* arg);

void app_main() {

    const char* imu_labels[] = {"Ring", "Index", "Middle", "Thumb", "BOH", "Pinky"}; // Labels for IMUs
    
    vTaskDelay(pdMS_TO_TICKS(10000)); //10 seconds delay before starting the timer

    //init_uart_for_input(); // Initialize UART for input
    //ESP_LOGI(TAG, "UART initialized for input");

    ESP_LOGI(TAG, "Initializing SPI");
    ESP_ERROR_CHECK(spi_init());

    ESP_LOGI(TAG, "Initializing IMUs");
    for (int i = 0; i < 6; i++) {
        setup_imu(imu_handles[i]);
    }

    // Set up Data Structures for IMU data
    for (int i = 0; i < 6; i++) {
        imu_data[i].data_index = 0; // Initialize data index for each IMU
        imu_data[i].spi = imu_handles[i]; // Assign SPI handle to each IMU data structure
        imu_data[i].label = imu_labels[i]; // Assign label to each IMU data structure
    }

    xTaskCreatePinnedToCore(read_imu_data_task, "imu_read", 6144, NULL, 5, &imu_read_task_handle, 1); // Create task to read IMU data (will be notified/started by timer)

    // Set up periodic data timer
    setup_data_timer();

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Main task does nothing, just keeps the app running
    }
    // Note: The FIFO reading is now handled by the timer callback
}

void read_imu_data_task(void* arg) {
    esp_task_wdt_add(NULL);  // Register this task with the Task Watchdog Timer
    bool running = false; // Flag to indicate if the task is running
    uint16_t sample = 0; // Sample index for circular buffer
    uint8_t cmd = 0; // Buffer for UART input

    // Configure the user button pin
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << USER_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf); // Configure the GPIO pin

    while (1) {
        if(running){
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for notification

            if(sample == 0) {
                printf("START SAMPLE\n"); // Print "START SAMPLE" to indicate the start of data output Matlab will read this and start recording data
            }
            for (uint8_t i = 0; i < 6; i++) {
                //ESP_LOGI(TAG, "Reading IMU %d", i);
                read_imu_data(imu_data, i); // Read and store data from each IMU
                //read_fifo_data(imu_data, i); // Read and store FIFO data from each IMU
            }

            if(sample >= SAMPLE_SIZE){
                sample = 0; // Reset sample index if it exceeds the buffer size
                running = false; // Stop the task after reading all IMUs for Sample Duration

                printf("START PRINT\n"); // Print "START PRINT" to indicate the start of data output Matlab will read this and start recording data
                for(uint8_t i = 0; i < 6; i++){
                    //print_imu_data(imu_data, i); // Print the data for each IMU
                    print_imu_data_csv(imu_data, i); // Print the data in CSV format for each IMU
                }
                printf("END\n"); // Print "END" to indicate the end of data output Matlab will read this and stop recording data
            }
            else{
                sample++; // Increment sample index (circular buffer)
            }
        }
        else{
            // // Check for UART input
            // uart_read_bytes(UART_PORT, &cmd, 1, portMAX_DELAY); // Read 1 byte from UART
            // ESP_LOGI(TAG, "Received UART char: %c (0x%02X)", cmd, cmd); // Log the received command
            // if (cmd == 's') { // Start command
            //     running = true; // Set running flag to true
            //     ESP_LOGI(TAG, "IMU data reading started");
            //     ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification to start reading
            // }

            // Check if the button is pressed (active low)
            if (gpio_get_level(USER_BUTTON_GPIO) == 0) {
                running = true;
                ESP_LOGI(TAG, "User button pressed: starting IMU data reading");
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            }
            esp_task_wdt_reset(); // Feed the watchdog to prevent timeout
            vTaskDelay(pdMS_TO_TICKS(100)); // Debounce delay
        }
    }
}