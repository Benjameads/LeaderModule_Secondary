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
#include "driver/gpio.h"
#include "relative_orientation.h"
#include <string.h>

static const float arrow_zp[3] = {0, 0, 1}; // Local Z+ axis
static const float axis_vectors[3][3] = {
    {1, 0, 0},  // AXIS_CURL   → rotation around X (finger curls)
    {0, 1, 0},  // AXIS_TWIST  → rotation around Y (finger twists)
    {0, 0, 1}   // AXIS_SPREAD → rotation around Z (fingers fan out)
};

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
        imu_orientation_detection(imu_state, orientation_data);

        // if(xQueueReceive(gestureQueue, &gesture_data, portMAX_DELAY) != pdTRUE) {
        //     ESP_LOGE(TAG, "Failed to receive gesture data from queue");
        // }

        // // Call the gesture functions to process the gesture data
        // disperse_state = disperse(gesture_data);
    }
}

void imu_orientation_detection(IMUState* imu_state, OrientationDatalist* orientation_data) {
    //static IMUOrientation last_orientation[NUMBER_OF_IMUS] = {IMU_FLAT_UP, IMU_FORWARD, FINGER_STRAIGHT, FINGER_STRAIGHT, FINGER_STRAIGHT, FINGER_STRAIGHT}; // Store the last orientation for each IMU
    int data_index = orientation_data[0].data_index; // Get the current data index
    static int count = 0;

    // Detect the orientation of each IMU based on the current orientation data
    
    // --- Back of Hand (IMU 0) Orientation ---
    float arrow_zp[3] = {0, 0, 1};  // Local Z+ axis
    float boh_y_world[3] = {0.0f, 0.0f, 0.0f}; // Initialize the BOH world vector
    quaternion_rotate_vector(orientation_data[BOH].data[data_index].quaternion, arrow_zp, boh_y_world);

    gpio_set_level(DEBUGPIN, 1); // Set the debug pin high to indicate sample processing start

    // Threshold constants
    #define VERTICAL_THRESHOLD 0.75f  // > ~41.4° tilt
    #define HORIZONTAL_TOLERANCE 0.5f // within ~60°

    if (boh_y_world[2] > VERTICAL_THRESHOLD) {
        imu_state[BOH].orientation = IMU_FLAT_UP;  // Fingers pointing up
    } else if (boh_y_world[2] < -VERTICAL_THRESHOLD) {
        imu_state[BOH].orientation = IMU_FLAT_DOWN;  // Fingers pointing down
    } else if (boh_y_world[1] > VERTICAL_THRESHOLD) {
        imu_state[BOH].orientation = IMU_FORWARD;  // Fingers pointing forward
    } else if (boh_y_world[1] < -VERTICAL_THRESHOLD) {
        imu_state[BOH].orientation = IMU_BACKWARD;  // Fingers pointing backward
    } else if (boh_y_world[0] > VERTICAL_THRESHOLD) {
        imu_state[BOH].orientation = IMU_RIGHT;  // Fingers pointing right
    } else if (boh_y_world[0] < -VERTICAL_THRESHOLD) {
        imu_state[BOH].orientation = IMU_LEFT;  // Fingers pointing left
    } else {
        imu_state[BOH].orientation = IMU_FLAT_UP;  // Default fallback
    }

    float forward[3] = {0, 1, 0};  // forward = +Y in local IMU frame
    float world[3];
    quaternion_rotate_vector(orientation_data[BOH].data[data_index].quaternion, forward, world);

    // Compute heading with 90° correction
    float heading_rad = atan2f(world[0], world[1]);
    float heading_deg = heading_rad * (180.0f / M_PI);
    heading_deg -= 90.0f;
    if (heading_deg < 0) heading_deg += 360.0f;
    if (heading_deg >= 360.0f) heading_deg -= 360.0f;


    // --- Thumb (IMU 1) Orientation ---
    //float thumb_vector[3] = {0, 0, 1};  // Assume Z+ is thumb direction (adjust if needed)
    float thumb_world[3], boh_world[3];

    // Rotate thumb and BOH vectors into world space
    quaternion_rotate_vector(orientation_data[1].data[data_index].quaternion, arrow_zp, thumb_world);
    quaternion_rotate_vector(orientation_data[BOH].data[data_index].quaternion, arrow_zp, boh_world);

    // Compare thumb to BOH
    compare_orientation(boh_world, thumb_world, &imu_state[1].relative);

    // Thresholds for orientation behavior
    if (imu_state[1].relative.curl > TILT_THRESHOLD) {
        imu_state[1].orientation = IMU_FORWARD;
    } else if (imu_state[1].relative.curl < -TILT_THRESHOLD) {
        imu_state[1].orientation = IMU_BACKWARD;
    } else if (imu_state[1].relative.spread > TILT_THRESHOLD) {
        imu_state[1].orientation = IMU_RIGHT;
    } else if (imu_state[1].relative.spread < -TILT_THRESHOLD) {
        imu_state[1].orientation = IMU_LEFT;
    } else {
        imu_state[1].orientation = IMU_FLAT_UP;
    }

    // --- Fingers (IMUs 2-6) Orientation ---
    for (int i = 2; i < NUMBER_OF_IMUS; i++) {
        float finger_world[3];

        quaternion_rotate_vector(orientation_data[i].data[data_index].quaternion, arrow_zp, finger_world);
        compare_orientation(boh_world, finger_world, &imu_state[i].relative);

        if (fabsf(imu_state[i].relative.curl) > CURLED_THRESHOLD) {
            imu_state[i].orientation = FINGER_CURLED;
        } else {
            imu_state[i].orientation = FINGER_STRAIGHT;
        }
    }

    // if(count == 0) {
    //     // Print the BOH orientation
    //     printf("BOH Orientation: %s, x: %.3f, y: %.3f, z: %.3f \n", imu_orientation_str(imu_state[BOH].orientation), boh_y_world[0], boh_y_world[1], boh_y_world[2]);
    //     printf("Heading: %.2f degrees\n", heading_deg);
    //     printf("Index: %s, Curl: %.2f, Spread: %.2f, Twist: %.2f\n",
    //         imu_orientation_str(imu_state[2].orientation),
    //         imu_state[2].relative.curl,
    //         imu_state[2].relative.spread,
    //         imu_state[2].relative.twist);
    // }

    

    //Use a state machine to monitor each axis to detect changes in orientation    
    for (int i = 0; i < NUMBER_OF_IMUS; i++) {
        track_axis_motion_quat(i, AXIS_CURL, imu_state, orientation_data);
        track_axis_motion_quat(i, AXIS_SPREAD, imu_state, orientation_data);
        track_axis_motion_quat(i, AXIS_TWIST, imu_state, orientation_data);
        // axis_orientation_change(i, YAW, &imu_state[i], orientation_data);
        // axis_orientation_change(i, PITCH, &imu_state[i], orientation_data);
        // axis_orientation_change(i, ROLL, &imu_state[i], orientation_data);
    }

    // if(AXIS_PEAKED == imu_state[BOH].axis[AXIS_SPREAD].state) {
    //     printf("Spread Movement Detected: %.3f\n", 
    //         RAD2DEG(imu_state[BOH].axis[AXIS_SPREAD].angle_diff));
    // }
    // if(AXIS_PEAKED == imu_state[BOH].axis[AXIS_CURL].state) {
    //     printf("Curl Movement Detected: %.3f\n", 
    //         RAD2DEG(imu_state[BOH].axis[AXIS_CURL].angle_diff));
    // }
    // if(AXIS_PEAKED == imu_state[BOH].axis[AXIS_TWIST].state) {
    //     printf("Twist Movement Detected: %.3f\n", 
    //         RAD2DEG(imu_state[BOH].axis[AXIS_TWIST].angle_diff));
    // }
    // if(count == 0) {
    //     printf("BOH Orientation: %s, x: %.3f, y: %.3f, z: %.3f \n", imu_orientation_str(imu_state[BOH].orientation), boh_y_world[0], boh_y_world[1], boh_y_world[2]);
    // }
    // if(count == 0){
    //     printf("BOH Spread State: %s\n", 
    //         axis_state_str(imu_state[BOH].axis[AXIS_SPREAD].state));
    // }
    // if (count == 0) {
    //     AxisState s = imu_state[BOH].axis[AXIS_SPREAD].state;
    //     printf("Raw Axis State Enum = %d\n", s);
    // }    

    count++;
    if (count >= 50) {
        count = 0; // Reset the count after processing all samples
    }

    gpio_set_level(DEBUGPIN, 0); // Set the debug pin low to indicate sample processing end
}

// This function uses quaternion orientation to detect movement (curl, splay, twist) on any axis
// It tracks the angular velocity between world vectors rotated from local IMU axes
// to identify significant movement start, peak, and change direction (i.e., gestures)
// Lookup table for reference vectors corresponding to anatomical axes
void track_axis_motion_quat(int imu_index, RelativeAxis which_axis, IMUState* imu_state, OrientationDatalist* orientation_data) {
    float v1[3], v2[3];
    float* q_prev;
    float* q_curr;
    int cur = orientation_data[imu_index].data_index;
    int past = (cur + SAMPLE_SIZE_ORIENTATION - 1) % SAMPLE_SIZE_ORIENTATION;

    q_prev = orientation_data[imu_index].data[past].quaternion; // Quaternion 1 sample ago
    q_curr = orientation_data[imu_index].data[cur].quaternion; // Current quaternion

    // Rotate the axis to world frame
    quaternion_rotate_vector(q_prev, arrow_zp, v1);
    quaternion_rotate_vector(q_curr, arrow_zp, v2);

    // Project onto axis-of-interest's orthogonal plane
    float v1_proj[3], v2_proj[3];
    project_onto_plane(v1, axis_vectors[which_axis], v1_proj);
    project_onto_plane(v2, axis_vectors[which_axis], v2_proj);

    // Normalize before angle math
    normalize_vector(v1_proj);
    normalize_vector(v2_proj);

    float angle = angle_between_projected_vectors(v1_proj, v2_proj, axis_vectors[which_axis]);
    float raw_signed_velocity = (angle / SAMPLE_PERIOD_SEC); // Angle in radians per second
    
    AxisTracker* tracker = &imu_state[imu_index].axis[which_axis];

    // Apply exponential moving average to velocity
    tracker->smoothed_velocity = (1.0f - EMA_ALPHA) * tracker->smoothed_velocity + EMA_ALPHA * raw_signed_velocity;
    tracker->angle_velocity = fabsf(tracker->smoothed_velocity); // optional for logging

    switch (tracker->state) {
        case AXIS_STILL:
            if (fabsf(tracker->smoothed_velocity) > VELOCITY_THRESHOLD) {
                tracker->state = tracker->smoothed_velocity > 0 ? AXIS_INCREASING : AXIS_DECREASING;
                tracker->start_vector[0] = v1_proj[0]; // Save the start vector
                tracker->start_vector[1] = v1_proj[1]; // Save the start vector
                tracker->start_vector[2] = v1_proj[2]; // Save the start vector
                tracker->reversal_counter = 0;
            }
            break;

        case AXIS_INCREASING:
            if (tracker->smoothed_velocity < VELOCITY_THRESHOLD) {
                tracker->state = AXIS_PEAKING;
                tracker->restore_state = AXIS_INCREASING; // Save the state for restoration
            } else {
                tracker->reversal_counter = 0;
                tracker->peak_vector[0] = v2_proj[0]; // Update peak vector
                tracker->peak_vector[1] = v2_proj[1]; // Update peak vector
                tracker->peak_vector[2] = v2_proj[2]; // Update peak vector
            }
            break;

        case AXIS_DECREASING:
            if (tracker->smoothed_velocity > -VELOCITY_THRESHOLD) {
                tracker->state = AXIS_PEAKING;
                tracker->restore_state = AXIS_DECREASING; // Save the state for restoration
            } else {
                tracker->reversal_counter = 0;
                tracker->peak_vector[0] = v2_proj[0]; // Update peak vector
                tracker->peak_vector[1] = v2_proj[1]; // Update peak vector
                tracker->peak_vector[2] = v2_proj[2]; // Update peak vector
            }
            break;

        case AXIS_PEAKING:
            if(tracker->reversal_counter < STALLLIMIT) {
                tracker->reversal_counter++;
                if(tracker->restore_state == AXIS_INCREASING && tracker->smoothed_velocity > VELOCITY_THRESHOLD) {
                    tracker->state = AXIS_INCREASING; // Continue increasing
                    tracker->restore_state = AXIS_STILL; // Reset restore state
                } else if(tracker->restore_state == AXIS_DECREASING && tracker->smoothed_velocity < -VELOCITY_THRESHOLD) {
                    tracker->state = AXIS_DECREASING; // Continue decreasing
                    tracker->restore_state = AXIS_STILL; // Reset restore state
                }
            } else {
                float possible_peak = angle_between_projected_vectors(tracker->start_vector, tracker->peak_vector, axis_vectors[which_axis]); // Calculate angle difference
                if (fabsf(possible_peak) > DEG2RAD(10.0f)) { // Arbitrary threshold to avoid noise
                    tracker->angle_diff = possible_peak; // Update angle difference
                    tracker->state = AXIS_PEAKED; // Set state to peaked
                } else {
                    tracker->state =  AXIS_STILL; // Restore to previous state
                    tracker->reversal_counter = 0; // Reset reversal counter
                    tracker->smoothed_velocity = 0.0f; // Reset smoothed velocity
                    tracker->start_vector[0] = 0.0f; // Reset start vector
                    tracker->start_vector[1] = 0.0f; // Reset start vector
                    tracker->start_vector[2] = 0.0f; // Reset start vector
                    tracker->peak_vector[0] = 0.0f; // Reset peak vector
                    tracker->peak_vector[1] = 0.0f; // Reset peak vector
                    tracker->peak_vector[2] = 0.0f; // Reset peak vector
                    tracker->angle_diff = 0.0f; // Reset angle difference
                    tracker->angle_velocity = 0.0f; // Reset angle velocity
                    tracker->smoothed_velocity = 0.0f; // Reset smoothed velocity
                }
            }
            break;

        case AXIS_PEAKED:
            tracker->state = AXIS_STILL;
            tracker->start_vector[0] = 0.0f; // Reset start vector
            tracker->start_vector[1] = 0.0f; // Reset start vector
            tracker->start_vector[2] = 0.0f; // Reset start vector
            tracker->peak_vector[0] = 0.0f; // Reset peak vector
            tracker->peak_vector[1] = 0.0f; // Reset peak vector
            tracker->peak_vector[2] = 0.0f; // Reset peak vector
            tracker->angle_diff = 0.0f;
            tracker->angle_velocity = 0.0f;
            tracker->smoothed_velocity = 0.0f;
            tracker->reversal_counter = 0;
            break;

        default:
            tracker->state = AXIS_STILL;
            tracker->reversal_counter = 0;
            tracker->smoothed_velocity = 0.0f;
            break;
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
// void axis_orientation_change(int imu_index, Axis axis, IMUState* imu_state, OrientationDatalist* orientation_data) {
//     int cur = orientation_data[imu_index].data_index;
//     int past = (cur + 1) % SAMPLE_SIZE_ORIENTATION; // 10 samples ago

//     //angle past is 10 samples ago
//     float angle_cur, angle_past;

//     // Select the appropriate axis from OrientationData
//     switch (axis) {
//         case YAW:
//             angle_cur = orientation_data[imu_index].data[cur].yaw;
//             angle_past = orientation_data[imu_index].data[past].yaw;
//             break;
//         case PITCH:
//             angle_cur = orientation_data[imu_index].data[cur].pitch;
//             angle_past = orientation_data[imu_index].data[past].pitch;
//             break;
//         case ROLL:
//             angle_cur = orientation_data[imu_index].data[cur].roll;
//             angle_past = orientation_data[imu_index].data[past].roll;
//             break;
//         default:
//             return;
//     }

//     AxisTracker* tracker = &imu_state[imu_index].axis[axis];
//     float velocity = shortest_angle_diff(angle_cur, angle_past) / ANGLE_VELOCITY_DT; // deg/s
//     tracker->angle_velocity = velocity;

//     switch (tracker->state) {
//         case AXIS_STILL:
//             if (velocity > VELOCITY_THRESHOLD) {
//                 tracker->state = AXIS_INCREASING;
//                 tracker->start_angle = peak_angle(imu_index, axis, orientation_data, FIND_MIN); // Find the min angle in the last 10 samples
//             } else if (velocity < -VELOCITY_THRESHOLD) {
//                 tracker->state = AXIS_DECREASING;
//                 tracker->start_angle = peak_angle(imu_index, axis, orientation_data, FIND_MAX); // Find the max angle in the last 10 samples
//             }
//             break;

//         case AXIS_INCREASING:
//             if (velocity <= 0.0f) {
//                 tracker->state = AXIS_PEAKED;
//                 tracker->peak_angle = peak_angle(imu_index, axis, orientation_data, FIND_MAX); // Find the max angle in the last 10 samples
//                 tracker->angle_diff = shortest_angle_diff(tracker->peak_angle, tracker->start_angle);
//             } else {
//                 tracker->peak_angle = angle_cur;
//             }
//             break;

//         case AXIS_DECREASING:
//             if (velocity >= 0.0f) {
//                 tracker->state = AXIS_PEAKED;
//                 tracker->peak_angle = peak_angle(imu_index, axis, orientation_data, FIND_MIN); // Find the min angle in the last 10 samples
//                 tracker->angle_diff = shortest_angle_diff(tracker->start_angle, tracker->peak_angle);
//             } else {
//                 tracker->peak_angle = angle_cur;
//             }
//             break;

//         case AXIS_PEAKED:
//             tracker->state = AXIS_STILL;
//             tracker->start_angle = 0.0f;
//             tracker->peak_angle = 0.0f;
//             tracker->angle_diff = 0.0f;
//             tracker->angle_velocity = 0.0f;
//             break;
//     }
// }

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

const char* imu_orientation_str(IMUOrientation o) {
    switch (o) {
        case IMU_FLAT_UP:    return "Flat Up";
        case IMU_FLAT_DOWN:  return "Flat Down";
        case IMU_LEFT:       return "Left Tilt";
        case IMU_RIGHT:      return "Right Tilt";
        case IMU_FORWARD:    return "Forward Tilt";
        case IMU_BACKWARD:   return "Backward Tilt";
        case FINGER_STRAIGHT:return "Straight";
        case FINGER_CURLED:  return "Curled";
        default:             return "Unknown";
    }
}

const char* axis_state_str(AxisState s) {
    switch (s) {
        case AXIS_STILL:     return "Still";
        case AXIS_INCREASING:return "Increasing";
        case AXIS_DECREASING:return "Decreasing";
        case AXIS_PEAKED:    return "Peaked";
        default:             return "Unknown";
    }
}

