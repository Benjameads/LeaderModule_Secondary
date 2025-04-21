#include <stdio.h>
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "orientation_task.h"
#include "imu_orientation.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "imu_read.h"
#include "imu_spi.h"
#include "MadgwickAHRS.h"


static OrientationDatalist orientation_data[NUMBER_OF_IMUS];

void imu_data_queue_init(void) {
    imuQueue = xQueueCreate(10, sizeof(struct IMUDatalist*)); // Create a queue to hold pointers to IMUDatalist structs
}

void gesture_queue_init(void) {
    gestureQueue = xQueueCreate(10, sizeof(GestureOrientationData*)); // Create a queue to hold pointers to GestureOrientationData structs
    orientationQueue = xQueueCreate(10, sizeof(OrientationDatalist*)); // Create a queue to hold pointers to OrientationDatalist structs
}

/**************************************************************************************************
    This task uses three data structures: IMUDatalist, OrientationDatalist, and GestureOrientationData
    The IMUDatalist holds the raw IMU data (passed via a queue, with data for one IMU at a time)
    The OrientationDatalist holds the Madgwick filter (used for orientation processing) and the filtered orientation data (5 sec, circular buffer) for each IMU
        The 5 seconds of data is not currently used, but is kept for future use
    The GestureOrientationData holds the current and previous orientation data (used for gesture processing)

    The IMUDatalist is received from the IMU read task (this task is blocked until data is received)
    The OrientationDatalist is updated using the IMU data and the Madgwick filter
    The GestureOrientationData is updated with current and previous orientation data for gesture processing
**************************************************************************************************/
void imu_orientation_worker_task(void* arg) {
    const float timeinterval = 1.0f / SAMPLE_RATE; // Time interval in seconds
    float t = 0.0f; // Initialize time variable
    //OrientationDatalist orientation_data[NUMBER_OF_IMUS];
    //static GestureOrientationData gesture_data[NUMBER_OF_IMUS]; // Array to hold gesture data for each IMU
    //GestureOrientationData* gesture_data_ptr = NULL; // Pointer to gesture data
    OrientationDatalist* orientation_data_ptr = NULL; // Pointer to orientation data

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
            update_imu_orientation_from_raw(&orientation_data[imu_index].filter , &imu_packet->data[0], 1.0f / SAMPLE_RATE, &yaw, &pitch, &roll);

            // //Update Gesture data for the gesture task
            // gesture_data[imu_index].previous.yaw = gesture_data[imu_index].current.yaw;
            // gesture_data[imu_index].previous.pitch = gesture_data[imu_index].current.pitch;
            // gesture_data[imu_index].previous.roll = gesture_data[imu_index].current.roll;
            // gesture_data[imu_index].current.yaw = yaw;
            // gesture_data[imu_index].current.pitch = pitch;
            // gesture_data[imu_index].current.roll = roll;

            // Wait for all IMUs to finish processing before sending gesture data to the queue
            // if(imu_index == NUMBER_OF_IMUS-1){
            //     // Send gesture data to the queue for gesture processing
            //     gesture_data_ptr = gesture_data;
            //     if (xQueueSend(gestureQueue, &gesture_data_ptr, portMAX_DELAY) != pdTRUE) {
            //         ESP_LOGE(TAG, "Failed to send gesture data to queue");
            //     }
            // }

            // Store output in orientation_data
            orientation_data[imu_index].data[data_index].yaw = yaw;
            orientation_data[imu_index].data[data_index].pitch = pitch;
            orientation_data[imu_index].data[data_index].roll = roll;
            for (int i = 0; i < 4; ++i){
                orientation_data[imu_index].data[data_index].quaternion[i] = orientation_data[imu_index].filter.q[i];
            }

            if(imu_index == NUMBER_OF_IMUS-1){
                // Send orientation data to the queue for printing
                orientation_data_ptr = orientation_data;
                if (xQueueSend(orientationQueue, &orientation_data_ptr, portMAX_DELAY) != pdTRUE) {
                    ESP_LOGE(TAG, "Failed to send orientation data to queue");
                }
            }

            if(data_index >= SAMPLE_SIZE_ORIENTATION-1){
                // if(imu_index == 5) {
                //     printf("%.2f,%d,%.3f,%.3f,%.3f\n",
                //         t,
                //         imu_index,
                //         orientation_data[imu_index].data[data_index].yaw,
                //         orientation_data[imu_index].data[data_index].pitch,
                //         orientation_data[imu_index].data[data_index].roll);
     
                //     t += timeinterval; // Increment time by the time interval
                // }

                //************************** OLD Code to print orientation data to CSV format ****************************//
                // if (imu_index == 0) {
                //     printf("START PRINT\n"); // Print "START PRINT" to indicate the start of data output Matlab will read this and start recording data
                // }
                // // Print the data in CSV format for each IMU
                // if (xSemaphoreTake(print_mutex, portMAX_DELAY)) {
                //     print_orientation_csv(&orientation_data[imu_index]);
                //     xSemaphoreGive(print_mutex);
                // }
                // if(imu_index == NUMBER_OF_IMUS-1){
                //     printf("END\n"); // Print "END" to indicate the end of data output Matlab will read this and stop recording data
                // }

                orientation_data[imu_index].data_index = 0; // Reset the index for the next set of data
            }
            else{
                orientation_data[imu_index].data_index++; // Increment data index (circular buffer)
            }

            //gpio_set_level(DEBUGPIN, 0); // Set the debug pin low to indicate sample processing end
        }
    }
}

// this needs to change to print just one imu, or needs to run when all IMUs are done
// If you print just one imu, you can print from the orientation task safely
//otherwise 5 of the IMU's orientation data may not have their last sample updated yet
// Print the orientation data in CSV format
void print_orientation_csv(OrientationDatalist* orientation_lists) {
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


