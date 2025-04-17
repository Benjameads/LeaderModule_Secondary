#include <stdio.h>
#include "esp_task_wdt.h"
#include "orientation_task.h"
#include "imu_orientation.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "imu_read.h"
#include "imu_spi.h"
#include "MadgwickAHRS.h"


static struct OrientationDatalist orientation_data[NUMBER_OF_IMUS];

void imu_data_queue_init(void) {
    imuQueue = xQueueCreate(10, sizeof(struct IMUDatalist*)); // Create a queue to hold pointers to IMUDatalist structs
}

void imu_orientation_worker_task(void* arg) {
    //struct OrientationDatalist orientation_data[NUMBER_OF_IMUS];
    //esp_task_wdt_add(NULL);  // Register this task with the Task Watchdog Timer

    for(int i = 0; i < NUMBER_OF_IMUS; i++){
        MadgwickInit(&orientation_data[i].filter); // Initialize the Madgwick filter for each IMU
        orientation_data[i].data_index = 0; // Initialize data index for each IMU
        orientation_data[i].imu_index = i; // Assign IMU index to each IMU data structure
    }

    while (1) {
        struct IMUDatalist* imu_packet = NULL;
        if (xQueueReceive(imuQueue, &imu_packet, portMAX_DELAY) == pdTRUE) {
            uint8_t imu_index = imu_packet->imu_index;
            int data_index = orientation_data[imu_index].data_index;

            //gpio_set_level(DEBUGPIN, 1); // Set the debug pin high to indicate sample processing start

            float yaw, pitch, roll;
            update_imu_orientation_from_raw(&orientation_data[imu_index].filter , &imu_packet->data[imu_packet->data_index], 1.0f / SAMPLE_RATE, &yaw, &pitch, &roll);

            // Store output in orientation_data
            orientation_data[imu_index].data[data_index].yaw = yaw;
            orientation_data[imu_index].data[data_index].pitch = pitch;
            orientation_data[imu_index].data[data_index].roll = roll;
            for (int i = 0; i < 4; ++i){
                orientation_data[imu_index].data[data_index].quaternion[i] = orientation_data[imu_index].filter.q[i];
            }

            if(data_index >= SAMPLE_SIZE_ORIENTATION-1){
                if (imu_index == 0) {
                    printf("START PRINT\n"); // Print "START PRINT" to indicate the start of data output Matlab will read this and start recording data
                }
                // Print the data in CSV format for each IMU
                if (xSemaphoreTake(print_mutex, portMAX_DELAY)) {
                    print_orientation_csv(&orientation_data[imu_index]);
                    xSemaphoreGive(print_mutex);
                }
                orientation_data[imu_index].data_index = 0; // Reset the index for the next set of data
                if(imu_index == NUMBER_OF_IMUS-1){
                    printf("END\n"); // Print "END" to indicate the end of data output Matlab will read this and stop recording data
                }
            }
            else{
                orientation_data[imu_index].data_index++; // Increment data index (circular buffer)
            }

            //gpio_set_level(DEBUGPIN, 0); // Set the debug pin low to indicate sample processing end
        }
        //esp_task_wdt_reset(); // Feed the watchdog to prevent timeout
        //vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// this needs to change to print just one imu, or needs to run when all IMUs are done
// If you print just one imu, you can print from the orientation task safely
//otherwise 5 of the IMU's orientation data may not have their last sample updated yet
// Print the orientation data in CSV format
void print_orientation_csv(struct OrientationDatalist* orientation_lists) {
    float timeinterval = 1.0f / SAMPLE_RATE; // Time interval in seconds
    float t = 0.0f; // Initialize time variable

    //printf("time,imu_id,yaw,pitch,roll\n");

    //for (int i = 0; i < imu_count; i++) {
        for (int i = 0; i < SAMPLE_SIZE_ORIENTATION; i++) {
            printf("%.2f,%d,%.3f,%.3f,%.3f\n",
                   t,
                   orientation_lists->imu_index,
                   orientation_lists->data[i].yaw,
                   orientation_lists->data[i].pitch,
                   orientation_lists->data[i].roll);

            t += timeinterval; // Increment time by the time interval

            if (i % 50 == 0) {
                //esp_task_wdt_reset();  // Feed the watchdog every 50 samples
                vTaskDelay(pdMS_TO_TICKS(10));  // Let other tasks run
            }
        }
    //}
}


