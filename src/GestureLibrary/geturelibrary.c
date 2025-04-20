#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "gesturelibrary.h"
#include "orientation_task.h"
#include "imu_read.h"
#include "imu_spi.h"

void gesture_worker_task(void* arg) {
    // GestureOrientationData* gesture_data = NULL;
    // GestureState disperse_state; // Initialize the gesture state
    OrientationDatalist* orientation_data = NULL;
    IMUState imu_state[NUMBER_OF_IMUS]; // Array to hold IMU states for each IMU

    while (1) {
        if(xQueueReceive(orientationQueue, &orientation_data, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to receive orientation data from queue");
        }

        // Call the function to detect the orientation of each IMU
        imu_orientaion_detection(imu_state, orientation_data);

        // if(xQueueReceive(gestureQueue, &gesture_data, portMAX_DELAY) != pdTRUE) {
        //     ESP_LOGE(TAG, "Failed to receive gesture data from queue");
        // }

        // // Call the gesture functions to process the gesture data
        // disperse_state = disperse(gesture_data);
    }
}

void imu_orientaion_detection(IMUState* imu_state, OrientationDatalist* orientation_data) {
    int data_index = orientation_data[0].data_index; // Get the current data index
    // Detect the orientation of each IMU based on the current orientation data

    //Fingers are IMUs 2-6, 0 is BOH, and 1 is Thumb
    for (int i = 2; i < NUMBER_OF_IMUS; i++) {
        float relative_pitch = fabsf(shortest_angle_diff(orientation_data[i].data[data_index].pitch, orientation_data[BOH].data[data_index].pitch));
        if ( relative_pitch > CURLED_THRESHOLD) {
            imu_state[i].orientation = FINGER_CURLED;
        } else {
            imu_state[i].orientation = FINGER_STRAIGHT;
        }
    }


    //Use a state machine to monitor each axis to detect changes in orientation
    for (int i = 0; i < NUMBER_OF_IMUS; i++) {
        axis_orientation_change(i, YAW, &imu_state[i], orientation_data);
        axis_orientation_change(i, PITCH, &imu_state[i], orientation_data);
        axis_orientation_change(i, ROLL, &imu_state[i], orientation_data);
    }
}

// This function is used to detect changes in orientation for a specific axis and IMU
// And therefore will be called for each axis (YAW, PITCH, ROLL) and for each IMU
// The passed in IMUState struct will be updated with the current state of the axis
    // If the axis is still the axis angles will be set to 0
    // If the axis is increasing or decreasing the axis will monitor the angles for a peak detected by calculating angular velocity
    // If the axis state has peaked the axis angles will be updated with the angle difference from the start of a movement to the peak of the movement
    // Each axis state will be set accordingly
// It will be configured in state machine style logic
void axis_orientation_change(int imu_index, Axis axis, IMUState* imu_state, OrientationDatalist* orientation_data) {
    int cur = orientation_data[imu_index].data_index;
    int past = (cur + 1) % SAMPLE_SIZE_ORIENTATION; // 10 samples ago

    //angle past is 10 samples ago
    float angle_cur, angle_prev, angle_past;

    // Select the appropriate axis from OrientationData
    switch (axis) {
        case YAW:
            angle_cur = orientation_data[imu_index].data[cur].yaw;
            angle_past = orientation_data[imu_index].data[past].yaw;
            break;
        case PITCH:
            angle_cur = orientation_data[imu_index].data[cur].pitch;
            angle_past = orientation_data[imu_index].data[past].pitch;
            break;
        case ROLL:
            angle_cur = orientation_data[imu_index].data[cur].roll;
            angle_past = orientation_data[imu_index].data[past].roll;
            break;
        default:
            return;
    }

    AxisTracker* tracker = &imu_state[imu_index].axis[axis];
    float velocity = shortest_angle_diff(angle_cur, angle_past) / ANGLE_VELOCITY_DT; // deg/s
    tracker->angle_velocity = velocity;

    switch (tracker->state) {
        case AXIS_STILL:
            if (velocity > VELOCITY_THRESHOLD) {
                tracker->state = AXIS_INCREASING;
                tracker->start_angle = peak_angle(imu_index, axis, orientation_data, FIND_MIN); // Find the min angle in the last 10 samples
            } else if (velocity < -VELOCITY_THRESHOLD) {
                tracker->state = AXIS_DECREASING;
                tracker->start_angle = peak_angle(imu_index, axis, orientation_data, FIND_MAX); // Find the max angle in the last 10 samples
            }
            break;

        case AXIS_INCREASING:
            if (velocity <= 0.0f) {
                tracker->state = AXIS_PEAKED;
                tracker->peak_angle = peak_angle(imu_index, axis, orientation_data, FIND_MAX); // Find the max angle in the last 10 samples
                tracker->angle_diff = shortest_angle_diff(tracker->peak_angle, tracker->start_angle);
            } else {
                tracker->peak_angle = angle_cur;
            }
            break;

        case AXIS_DECREASING:
            if (velocity >= 0.0f) {
                tracker->state = AXIS_PEAKED;
                tracker->peak_angle = peak_angle(imu_index, axis, orientation_data, FIND_MIN); // Find the min angle in the last 10 samples
                tracker->angle_diff = shortest_angle_diff(tracker->start_angle, tracker->peak_angle);
            } else {
                tracker->peak_angle = angle_cur;
            }
            break;

        case AXIS_PEAKED:
            tracker->state = AXIS_STILL;
            tracker->start_angle = 0.0f;
            tracker->peak_angle = 0.0f;
            tracker->angle_diff = 0.0f;
            tracker->angle_velocity = 0.0f;
            break;
    }
}

// This function is used to detect the peak angle for a specific axis and IMU within the window of 10 samples
float peak_angle(int imu_index, Axis axis, const OrientationDatalist* orientation_data, int max_boolean) {
    int cur_index = orientation_data[imu_index].data_index;
    float current_angle;

    // Select current angle as the reference
    switch(axis) {
        case YAW:
            current_angle = orientation_data[imu_index].data[cur_index].yaw;
            break;
        case PITCH:
            current_angle = orientation_data[imu_index].data[cur_index].pitch;
            break;
        case ROLL:
            current_angle = orientation_data[imu_index].data[cur_index].roll;
            break;
        default:
            return 0.0f;
    }

    float peak_angle = current_angle;
    float best_diff = max_boolean ? -360.0f : 360.0f;

    for (int i = 0; i < SAMPLE_SIZE_ORIENTATION; i++) {
        float sample_angle;

        switch(axis) {
            case YAW:
                sample_angle = orientation_data[imu_index].data[i].yaw;
                break;
            case PITCH:
                sample_angle = orientation_data[imu_index].data[i].pitch;
                break;
            case ROLL:
                sample_angle = orientation_data[imu_index].data[i].roll;
                break;
            default:
                return 0.0f;
        }

        float diff = shortest_angle_diff(sample_angle, current_angle);

        if ((max_boolean && diff > best_diff) || (!max_boolean && diff < best_diff)) {
            best_diff = diff;
            peak_angle = sample_angle;
        }
    }

    return peak_angle;
}


float wrap_angle_deg(float angle) {
    while (angle <= -180.0f) angle += 360.0f;
    while (angle > 180.0f) angle -= 360.0f;
    return angle;
}

float shortest_angle_diff(float a, float b) {
    float diff = wrap_angle_deg(a - b);
    return diff;
}
