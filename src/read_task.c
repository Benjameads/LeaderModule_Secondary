#include "read_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "imu_read.h"
#include "imu_spi.h"
#include "orientation_task.h"
#include "MadgwickAHRS.h"

void read_imu_data_task(void* arg) {
    struct IMUDatalist imu_data[NUMBER_OF_IMUS]; // Array to hold IMU data for each IMU
        //Currently this IMU Data list holds lists for all 6 IMUS, and only one set of data is stored in each list
        //We used to store more data but after orientation analysis we only need the latest data for each IMU

    //esp_task_wdt_add(NULL);  // Register this task with the Task Watchdog Timer
    bool running = false; // Flag to indicate if the task is running
    uint16_t sample = 0; // Sample index for circular buffer
    struct IMUDatalist* imu_packet = NULL; // Pointer to IMU data packet

    // Set up Data Structures for IMU data
    for (int i = 0; i < 6; i++) {
        imu_data[i].data_index = 0; // Initialize data index for each IMU
        imu_data[i].spi = imu_handles[i]; // Assign SPI handle to each IMU data structure
        imu_data[i].imu_index = i; // Assign IMU index to each IMU data structure
    }

    while (1) {
        if(running){
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for notification

            // Was used to MATLAB data output
            if(sample == 0) {
                printf("START SAMPLE\n"); // Print "START SAMPLE" to indicate the start of data output Matlab will read this and start recording data
            }

            for (uint8_t i = 0; i < 6; i++) {
                //ESP_LOGI(TAG, "Reading IMU %d", i);
                read_imu_data(imu_data, i); // Read and store data from each IMU
                imu_packet = &imu_data[i]; // Get the IMU data packet for the current IMU
                xQueueSendToBack(imuQueue, &imu_packet, portMAX_DELAY); // Send the data to the queue for processing
            }

            if(sample >= SAMPLE_SIZE_ORIENTATION-1){
                sample = 0; // Reset sample index if it exceeds the buffer size
                running = false; // Stop the task after reading all IMUs for Sample Duration

                // printf("START PRINT\n"); // Print "START PRINT" to indicate the start of data output Matlab will read this and start recording data
                // for(uint8_t i = 0; i < 6; i++){
                //     //print_imu_data(imu_data, i); // Print the data for each IMU
                //     print_imu_data_csv(imu_data, i); // Print the data in CSV format for each IMU
                // }
                // printf("END\n"); // Print "END" to indicate the end of data output Matlab will read this and stop recording data
            }
            else{
                sample++; // Increment sample index (circular buffer)
            }
        }
        else{
            // Check if the button is pressed (active low)
            if (gpio_get_level(USER_BUTTON_GPIO) == 0) {
                running = true;
                ESP_LOGI(TAG, "User button pressed: starting IMU data reading");
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                //This will need to be removed eventually, but is helpful for testing communication.
                send_gesture_byte('F'); //REMOVE THIS LINE FOR FINAL VERSION
                printf("Gesture byte sent: %c\n", 'F'); // REMOVE THIS LINE FOR FINAL VERSION
            }
            //esp_task_wdt_reset(); // Feed the watchdog to prevent timeout
            vTaskDelay(pdMS_TO_TICKS(10)); // Debounce delay
        }
    }
}

void setup_btn(void) {
    // Configure the user button pin
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << USER_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf); // Configure the GPIO pin
}