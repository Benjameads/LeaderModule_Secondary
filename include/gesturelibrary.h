#ifndef GESTURELIBRARY_H
#define GESTURELIBRARY_H

#include "orientation_task.h"

typedef enum {
    GESTURE_START,
    GESTURE_STEP_1,
    GESTURE_STEP_2,
    GESTURE_STALL,
    GESTURE_COMPLETE
} GestureState;

#define STALLLIMIT 5 // The limit for the stall state to be considered a valid gesture

#pragma once
#include <math.h> // for fabsf

#define FLAT_THRESHOLD 30.0f
#define ANGLE_VELOCITY_DT 0.1f      // 0.1 second window = 10 samples @ 100 Hz
#define CURLED_THRESHOLD 35.0f
#define VELOCITY_THRESHOLD 20.0f    // deg/s threshold to detect motion
#define STALL_LIMIT 5               // for debounce when movement reverses
#define FIND_MIN 0
#define FIND_MAX 1
#define FLAT_PITCH_TOLERANCE 20.0f
#define FLAT_ROLL_TOLERANCE 20.0f
#define ROLL_FLIP_THRESHOLD 160.0f
#define TILT_THRESHOLD 45.0f


#define BOH_IS_FLAT \
    (fabsf(gesture_data[BOH].current.pitch) < FLAT_THRESHOLD && \
    fabsf(gesture_data[BOH].current.roll) < FLAT_THRESHOLD)

#define IS_FLAT(index) \
    (fabsf(gesture_data[(index)].current.pitch) < FLAT_THRESHOLD && \
    fabsf(gesture_data[(index)].current.roll) < FLAT_THRESHOLD)

#define YAWDIF(i)  shortest_angle_diff(gesture_data[(i)].current.yaw, gesture_data[(i)].previous.yaw)
#define PITCHDIF(i)  shortest_angle_diff(gesture_data[(i)].current.pitch, gesture_data[(i)].previous.pitch)
#define ROLLDIF(i)  shortest_angle_diff(gesture_data[(i)].current.roll, gesture_data[(i)].previous.roll)

#define STARTYAWDIF(i)  shortest_angle_diff(gesture_data[(i)].current.yaw, start_data[(i)].yaw)
#define STARTPITCHDIF(i)  shortest_angle_diff(gesture_data[(i)].current.pitch, start_data[(i)].pitch)
#define STARTROLLDIF(i)  shortest_angle_diff(gesture_data[(i)].current.roll, start_data[(i)].roll)

typedef enum {
    AXIS_STILL,
    AXIS_INCREASING,
    AXIS_DECREASING,
    AXIS_PEAKED
} AxisState;

typedef struct {
    AxisState state;
    float start_angle;
    float peak_angle;
    float angle_velocity;  // instantaneous ω
    float angle_diff;      // peak - start
} AxisTracker;

typedef enum {
    IMU_FLAT_UP,
    IMU_FLAT_DOWN,
    IMU_LEFT, // IMU is tilted to the left
    IMU_RIGHT, // IMU is tilted to the right
    IMU_FORWARD, // IMU is tilted forward
    IMU_BACKWARD, // IMU is tilted backward
    FINGER_STRAIGHT,
    FINGER_CURLED
} IMUOrientation;

typedef enum {
    YAW,
    PITCH,
    ROLL
} Axis;

typedef struct {
    IMUOrientation orientation;
    AxisTracker axis[3]; // yaw, pitch, roll
} IMUState;

// The data structure for the gesture library is defined in orientation_task.h

float wrap_angle_deg(float angle);
float shortest_angle_diff(float a, float b);

void gesture_worker_task(void* arg);
void imu_orientation_detection(IMUState* imu_state, OrientationDatalist* orientation_data);
void axis_orientation_change(int imu_index, Axis axis, IMUState* imu_state, OrientationDatalist* orientation_data);
float peak_angle(int imu_index, Axis axis, const OrientationDatalist* orientation_data, int max_boolean);
float wrap_angle_deg(float angle);
float shortest_angle_diff(float a, float b);
const char* imu_orientation_str(IMUOrientation o);


GestureState disperse(GestureOrientationData* gesture_data);

#endif
