#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_read.h"
#include "imu_spi.h"
#include "orientation_task.h"
#include "MadgwickAHRS.h"

struct OrientationDatalist orientation_data[NUMBER_OF_IMUS];

Madgwick filters[NUMBER_OF_IMUS]; // Array of Madgwick filters for each IMU

void imu_data_queue_init() {
    imuQueue = xQueueCreate(10, sizeof(struct IMUDatalist*)); // Create a queue to hold pointers to IMUDatalist structs
}

void imu_orientation_worker_task(void* arg) {
    for(int i = 0; i < NUMBER_OF_IMUS; i++){
        MadgwickInit(&filters[i]);
    }

    while (1) {
        struct IMUDatalist* imu_packet = NULL;
        if (xQueueReceive(imuQueue, &imu_packet, portMAX_DELAY) == pdTRUE) {
            int imu_index = imu_packet - imu_data;  // assuming imu_data[] is global
            struct IMUData* sample = &imu_packet->data[imu_packet->data_index];
            struct OrientationDatalist* out = &orientation_data[imu_index];

            float yaw, pitch, roll;
            update_imu_orientation_from_raw(&filter, sample, 1.0f / SAMPLE_RATE, &yaw, &pitch, &roll);

            // Store output
            int idx = out->data_index;
            out->data[idx].yaw = yaw;
            out->data[idx].pitch = pitch;
            out->data[idx].roll = roll;

            for (int i = 0; i < 4; ++i)
                out->data[idx].quaternion[i] = filter.q[i];

            out->data_index = (idx + 1) % SAMPLE_SIZE;
        }
    }
}

