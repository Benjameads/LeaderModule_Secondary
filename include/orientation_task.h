#ifndef ORIENTATION_TASK_H
#define ORIENTATION_TASK_H

#include "MadgwickAHRS.h"

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
    int imu_index;               // index of the IMU this data belongs to
    Madgwick filter;             // Madgwick filter for this IMU
};

extern QueueHandle_t imuQueue; // holds pointers to IMUDatalist structs
extern SemaphoreHandle_t print_mutex; // Mutex for printing orientation data
// extern struct OrientationDatalist orientation_data[NUMBER_OF_IMUS]; // Array to hold orientation data for each IMU

void imu_orientation_worker_task(void* arg);
void print_orientation_csv(struct OrientationDatalist* orientation_lists);
void imu_data_queue_init(void);

#endif