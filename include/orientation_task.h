#ifndef ORIENTATION_TASK_H
#define ORIENTATION_TASK_H

#include "driver/spi_master.h"
#include "imu_read.h"


#define SAMPLE_RATE 100 // Sample rate in Hz
#define SAMPLE_DURATION 5 // 5 seconds of data
#define SAMPLE_SIZE_ORIENTATION (SAMPLE_RATE * SAMPLE_DURATION) // Number of samples to read

struct OrientationData {
    float yaw;
    float pitch;
    float roll;
    float quaternion[4]; // [w, x, y, z]
};

struct OrientationDatalist {
    struct OrientationData data[SAMPLE_SIZE_ORIENTATION];
    int data_index;               // current sample index
    const char* label;            // IMU name
};

extern QueueHandle_t imuQueue; // holds pointers to IMUDatalist structs
extern struct OrientationDatalist orientation_data[NUMBER_OF_IMUS]; // Array to hold orientation data for each IMU

void imu_orientation_worker_task(void* arg);

#endif