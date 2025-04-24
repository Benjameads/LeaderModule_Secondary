#ifndef GESTURELIBRARY_H
#define GESTURELIBRARY_H

#include "orientation_task.h"
#include "relative_orientation.h"
#include "espnow_comm.h"
#include "imu_read.h"
#include "quaternion_utils.h"

typedef enum {
    GESTURE_INCOMPLETE,
    GESTURE_COMPLETE
} GestureState;

#define STALLLIMIT 5 // The limit for the stall state to be considered a valid gesture

#pragma once
#include <math.h> // for fabsf

#define CURLED_THRESHOLD 0.7f
#define VELOCITY_THRESHOLD 1.0f    // rad/s threshold to detect motion
#define TILT_THRESHOLD 45.0f
#define EMA_ALPHA          0.1f   // smoothing factor for exponential moving average
#define SAMPLE_PERIOD_SEC 0.01f // 10ms sample period

#define RAD2DEG(x) ((x) * 57.2957795f)

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
    AXIS_PEAKING,
    AXIS_PEAKED
} AxisState;

typedef struct {
    float angle_velocity;
    float smoothed_velocity;  // NEW: for EMA smoothing
    float start_angle;
    float peak_velocity;      // Track peak smoothed velocity
    float angle_diff;
    int reversal_counter;
    AxisState state;
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

typedef enum {
    AXIS_CURL = 0,
    AXIS_SPREAD = 1,
    AXIS_TWIST = 2
} RelativeAxis;

typedef struct {
    IMUOrientation orientation;
    AxisTracker axis[3]; // curl, spread, twist
    RelativeRotation relative;
} IMUState;

// The data structure for the gesture library is defined in orientation_task.h

float wrap_angle_deg(float angle);
float shortest_angle_diff(float a, float b);

void gesture_worker_task(void* arg);
void imu_orientation_detection(IMUState* imu_state, OrientationDatalist* orientation_data);
//void axis_orientation_change(int imu_index, Axis axis, IMUState* imu_state, OrientationDatalist* orientation_data);
void track_axis_motion_quat(int imu_index, RelativeAxis axis, IMUState* imu_state, OrientationDatalist* orientation_data);
//void track_axis_motion_quat(int imu_index, RelativeAxis which_axis, IMUState* imu_state, const float q_diff[4]);
float peak_angle(int imu_index, Axis axis, const OrientationDatalist* orientation_data, int max_boolean);
float wrap_angle_deg(float angle);
float shortest_angle_diff(float a, float b);
const char* imu_orientation_str(IMUOrientation o);
const char* axis_state_str(AxisState s);

GestureState the_bird(IMUState* imu_state);
//GestureState disperse(GestureOrientationData* gesture_data);
//GestureState freeze(GestureOrientationData* gesture_data);

#endif
