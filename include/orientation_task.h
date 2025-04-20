#ifndef ORIENTATION_TASK_H
#define ORIENTATION_TASK_H

#include "MadgwickAHRS.h"
#include "Freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define DEBUGPIN GPIO_NUM_1 // GPIO pin for debugging

#define SAMPLE_RATE 100 // Sample rate in Hz
#define SAMPLE_DURATION 5 // 5 seconds of data
//#define SAMPLE_SIZE_ORIENTATION (SAMPLE_RATE * SAMPLE_DURATION) // Number of samples to read
#define SAMPLE_SIZE_ORIENTATION 10 // Number of samples to read

struct OrientationData {
    float yaw;      // Yaw angle in degrees
    float pitch;    // Pitch angle in degrees
    float roll;     // Roll angle in degrees
    float quaternion[4]; // [w, x, y, z]
};

typedef struct {
    struct OrientationData data[SAMPLE_SIZE_ORIENTATION];
    int data_index;               // current sample index
    int imu_index;               // index of the IMU this data belongs to
    Madgwick filter;             // Madgwick filter for this IMU
} OrientationDatalist;

// Gesture data structure to hold current and previous orientation data for gesture processing
// An array of these structs will be created, one element for each IMU
typedef struct {
    struct OrientationData current; // Array to hold current orientation data
    struct OrientationData previous; // Array to hold previous orientation data
}GestureOrientationData;

extern QueueHandle_t imuQueue; // holds pointers to IMUDatalist structs
extern QueueHandle_t gestureQueue; // holds pointers to GestureOrientationData structs
extern QueueHandle_t orientationQueue; // holds pointers to OrientationDatalist structs
extern SemaphoreHandle_t print_mutex; // Mutex for printing orientation data
// extern OrientationDatalist orientation_data[NUMBER_OF_IMUS]; // Array to hold orientation data for each IMU

void imu_orientation_worker_task(void* arg);
void gesture_queue_init(void);
void print_orientation_csv(OrientationDatalist* orientation_lists);
void imu_data_queue_init(void);

#endif
