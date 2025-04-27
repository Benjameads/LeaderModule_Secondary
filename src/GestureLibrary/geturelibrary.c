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

//static const float arrow_zp[3] = {0, 0, 1}; // Local Z+ axis
//static const float arrow_yp[3] = {0, 1, 0}; // Local Z+ axis
// static const float axis_vectors[3][3] = {
//     {1, 0, 0},  // AXIS_CURL   → rotation around X (finger curls)
//     {0, 1, 0},  // AXIS_TWIST  → rotation around Y (finger twists)
//     {0, 0, 1}   // AXIS_SPREAD → rotation around Z (fingers fan out)
// };
static const float x_local[3][3] = {
    {0, 1, 0},  // AXIS_CURL → Y+ (points forward from finger)
    {1, 0, 0}, // AXIS_TWIST → X+ (points right from finger)
    {1, 0, 0}   // AXIS_SPREAD → X+ (points right from finger)
};
static const float y_local[3][3] = {
    {0, 0, 1},  // AXIS_CURL → Z+ (points up from finger)
    {0, 0, 1},  // AXIS_TWIST → Z+ (points up from finger)
    {0, 1, 0}   // AXIS_SPREAD → Y+ (points forward from finger)
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
        disperse(imu_state); // Call the disperse function to check for the disperse gesture
        // freeze(imu_state); // Call the freeze function to check for the freeze gesture
        // one(imu_state); // Call the one function to check for the one gesture
        // two(imu_state); // Call the two function to check for the two gesture
        // three(imu_state); // Call the three function to check for the three gesture
        // four(imu_state); // Call the four function to check for the four gesture
        // five(imu_state); // Call the five function to check for the five gesture
        // six(imu_state); // Call the six function to check for the six gesture
        // seven(imu_state); // Call the seven function to check for the seven gesture
        // eight(imu_state); // Call the eight function to check for the eight gesture
        // nine(imu_state); // Call the nine function to check for the nine gesture
        // the_bird(imu_state); // Call the the_bird function to check for the bird gesture

    }
}

void imu_orientation_detection(IMUState* imu_state, OrientationDatalist* orientation_data) {
    //static IMUOrientation last_orientation[NUMBER_OF_IMUS] = {IMU_FLAT_UP, IMU_FORWARD, FINGER_STRAIGHT, FINGER_STRAIGHT, FINGER_STRAIGHT, FINGER_STRAIGHT}; // Store the last orientation for each IMU
    int data_index = orientation_data[0].data_index; // Get the current data index

    static int count = 0; // Counter for debugging

    // Detect the orientation of each IMU based on the current orientation data
    
    // --- Back of Hand (IMU 0) Orientation ---
    // Define local vectors
    float arrow_xp[3] = {1, 0, 0}; // Local X+ axis (thumb direction)
    float arrow_yp[3] = {0, 1, 0}; // Local Y+ axis (finger direction)
    float arrow_zp[3] = {0, 0, 1}; // Local Z+ axis (palm up direction)

    // Rotate into world frame
    float boh_x_world[3] = {0};
    float boh_y_world[3] = {0};
    float boh_z_world[3] = {0};
    quaternion_rotate_vector(orientation_data[BOH].data[data_index].quaternion, arrow_xp, boh_x_world);
    quaternion_rotate_vector(orientation_data[BOH].data[data_index].quaternion, arrow_yp, boh_y_world);
    quaternion_rotate_vector(orientation_data[BOH].data[data_index].quaternion, arrow_zp, boh_z_world);

    // Threshold constants
    #define VERTICAL_THRESHOLD 0.75f
    #define HORIZONTAL_THRESHOLD 0.5f

    gpio_set_level(DEBUGPIN, 1); // Start processing

    if (fabsf(boh_x_world[2]) < HORIZONTAL_THRESHOLD && fabsf(boh_y_world[2]) < HORIZONTAL_THRESHOLD && boh_z_world[2] > VERTICAL_THRESHOLD) {
        imu_state[BOH].orientation = IMU_FLAT_UP; 
    } else if (fabsf(boh_x_world[2]) < HORIZONTAL_THRESHOLD && fabsf(boh_y_world[2]) < HORIZONTAL_THRESHOLD && boh_z_world[2] < -VERTICAL_THRESHOLD) {
        imu_state[BOH].orientation = IMU_FLAT_DOWN;
    } else if (fabsf(boh_x_world[2]) < HORIZONTAL_THRESHOLD && boh_y_world[2] < -VERTICAL_THRESHOLD && fabsf(boh_z_world[2]) < HORIZONTAL_THRESHOLD ) {
        imu_state[BOH].orientation = IMU_FORWARD;
    } else if (fabsf(boh_x_world[2]) < HORIZONTAL_THRESHOLD && boh_y_world[2] > VERTICAL_THRESHOLD && fabsf(boh_z_world[2]) < HORIZONTAL_THRESHOLD) {
        imu_state[BOH].orientation = IMU_BACKWARD;
    } else if (boh_x_world[2] > VERTICAL_THRESHOLD && fabsf(boh_y_world[2]) < HORIZONTAL_THRESHOLD && fabsf(boh_z_world[2]) < HORIZONTAL_THRESHOLD) {
        imu_state[BOH].orientation = IMU_LEFT;
    } else if (boh_x_world[2] < -VERTICAL_THRESHOLD && fabsf(boh_y_world[2]) < HORIZONTAL_THRESHOLD && fabsf(boh_z_world[2]) < HORIZONTAL_THRESHOLD) {
        imu_state[BOH].orientation = IMU_RIGHT;
    } else {
        imu_state[BOH].orientation = IMU_ORIENTATION_UNKNOWN;  // Default fallback
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

    // // --- Thumb (IMU 1) Orientation ---

    quaternion_rotate_vector(orientation_data[THUMB].data[data_index].quaternion, forward, world);

    // Compute heading with 90° correction
    float thumb_heading_rad = atan2f(world[0], world[1]);
    float thumb_heading_deg = thumb_heading_rad * (180.0f / M_PI);
    thumb_heading_deg -= 90.0f;
    if (thumb_heading_deg < 0) thumb_heading_deg += 360.0f;
    if (thumb_heading_deg >= 360.0f) thumb_heading_deg -= 360.0f;

    float* q_boh;
    q_boh = orientation_data[BOH].data[data_index].quaternion;
    float* q_thumb;
    q_thumb = orientation_data[THUMB].data[data_index].quaternion; // Current quaternion for thumb IMU

    float thumb_x_local[3] = {0, 1, 0}; // Local X+ axis (Forward from BOH & forward from thumb nail)
    float thumb_y_local[3] = {-1, 0, 0}; // Local Y+ axis (Left from BOH & and left from thumb nail)

    float thumb_spread = rotation_about_axis(q_boh, q_thumb, thumb_x_local, thumb_y_local);
    //float thumb_curl = rotation_about_axis(q_boh, q_thumb, x_local[AXIS_CURL], y_local[AXIS_CURL]);

    if(thumb_spread > 0.15f) {
        imu_state[THUMB].orientation = THUMB_EXTENDED;
    } else if (thumb_spread < -0.5f) {
        imu_state[THUMB].orientation = THUMB_UNDER_PALM;
    } else {
        imu_state[THUMB].orientation = THUMB_AlIGNED;
    }

    //print thumb spread angle and boh heading
    // if(count == 0) {
    //     printf("IMU %d, Orientation: %s\n", THUMB, imu_orientation_str(imu_state[THUMB].orientation));
    // }
    // if(count == 0) {
    //     printf("IMU %d, Spread: %.3f, T_Heading: %.3f, B_Heading: %.3f\n", THUMB, thumb_spread, thumb_heading_deg, heading_deg);
    // }

    float* q_finger;
    // --- Fingers (IMUs 2-6) Orientation ---
    for (int i = 2; i < NUMBER_OF_IMUS; i++) {
        q_finger = orientation_data[i].data[data_index].quaternion; // Current quaternion for finger IMU

        imu_state[i].relative.curl = rotation_about_axis(q_boh, q_finger, x_local[AXIS_CURL], y_local[AXIS_CURL]);

        // if(count == 0 /*&& i == PINKY*/) {
        //     printf("IMU %d, Curl: %.3f\n", i, imu_state[i].relative.curl);
        // }

        if (fabsf(imu_state[i].relative.curl) > CURLED_THRESHOLD) {
            imu_state[i].orientation = FINGER_CURLED;
        } else {
            imu_state[i].orientation = FINGER_STRAIGHT;
        }
    }
    

    //Use a state machine to monitor each axis to detect changes in orientation    
    for (int i = 0; i < NUMBER_OF_IMUS; i++) {
        track_axis_motion_quat(i, AXIS_CURL, imu_state, orientation_data);
        track_axis_motion_quat(i, AXIS_SPREAD, imu_state, orientation_data);
        track_axis_motion_quat(i, AXIS_TWIST, imu_state, orientation_data);
        // axis_orientation_change(i, YAW, &imu_state[i], orientation_data);
        // axis_orientation_change(i, PITCH, &imu_state[i], orientation_data);
        // axis_orientation_change(i, ROLL, &imu_state[i], orientation_data);
    }

    // if(count == 0) {
    //     // Print the all relative orientations for debugging
    //     printf("BOH: %s, THUMB: %s, INDEX: %s, MIDDLE: %s, RING: %s, PINKY: %s\n", 
    //         imu_orientation_str(imu_state[BOH].orientation), 
    //         imu_orientation_str(imu_state[THUMB].orientation),
    //         imu_orientation_str(imu_state[INDEX].orientation),
    //         imu_orientation_str(imu_state[MIDDLE].orientation),
    //         imu_orientation_str(imu_state[RING].orientation),
    //         imu_orientation_str(imu_state[PINKY].orientation));
    // }

    if(AXIS_PEAKED == imu_state[BOH].axis[AXIS_SPREAD].state) {
        printf("Spread Movement Detected: %.3f\n", 
            RAD2DEG(imu_state[BOH].axis[AXIS_SPREAD].angle_diff));
    }
    if(AXIS_PEAKED == imu_state[BOH].axis[AXIS_CURL].state) {
        printf("Curl Movement Detected: %.3f\n", 
            RAD2DEG(imu_state[BOH].axis[AXIS_CURL].angle_diff));
    }
    if(AXIS_PEAKED == imu_state[BOH].axis[AXIS_TWIST].state) {
        printf("Twist Movement Detected: %.3f\n", 
            RAD2DEG(imu_state[BOH].axis[AXIS_TWIST].angle_diff));
    }
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
    float* q_prev;
    float* q_curr;
    int cur = orientation_data[imu_index].data_index;
    int past = (cur + SAMPLE_SIZE_ORIENTATION - 1) % SAMPLE_SIZE_ORIENTATION;

    q_prev = orientation_data[imu_index].data[past].quaternion; // Quaternion 1 sample ago
    q_curr = orientation_data[imu_index].data[cur].quaternion; // Current quaternion

    // This rotation function uses quaternions to calculate the angle between two quaternions
    // You give two vectors x_local and y_local, which define a plane orthogonal to the axis of interest
    float angle = rotation_about_axis(q_prev, q_curr, x_local[which_axis], y_local[which_axis]);
    float raw_signed_velocity = (angle / SAMPLE_PERIOD_SEC); // Angle in radians per second

    // static int count = 0;
    // int debug_imu = BOH;
    // if(count == 0 && imu_index == debug_imu && which_axis == AXIS_TWIST) {
    //     printf("IMU %d, Axis %d, q1=_prev: %.3f, %.3f, %.3f, %.3f\n", imu_index, which_axis, q_prev[0], q_prev[1], q_prev[2], q_prev[3]);
    //     printf("IMU %d, Axis %d, q2=_curr: %.3f, %.3f, %.3f, %.3f\n", imu_index, which_axis, q_curr[0], q_curr[1], q_curr[2], q_curr[3]);
    //     printf("IMU %d, Axis %d, Angle: %.3f, Raw Velocity: %.3f\n", imu_index, which_axis, angle, raw_signed_velocity);
    //     // printf("v1: %.3f, %.3f, %.3f\n", v1[0], v1[1], v1[2]);
    //     // printf("v2: %.3f, %.3f, %.3f\n", v2[0], v2[1], v2[2]);
    //     // printf("projected v1: %.3f, %.3f, %.3f\n", v1_proj[0], v1_proj[1], v1_proj[2]);
    //     // printf("projected v2: %.3f, %.3f, %.3f\n", v2_proj[0], v2_proj[1], v2_proj[2]);
    // }

    // if(imu_index == debug_imu && which_axis == AXIS_TWIST) {
    //     count = (count + 1) % 50; // Reset count every 50 samples
    // }
    
    AxisTracker* tracker = &imu_state[imu_index].axis[which_axis];

    // Apply exponential moving average to velocity
    tracker->smoothed_velocity = (1.0f - EMA_ALPHA) * tracker->smoothed_velocity + EMA_ALPHA * raw_signed_velocity;
    tracker->angle_velocity = fabsf(tracker->smoothed_velocity); // optional for logging

    switch (tracker->state) {
        case AXIS_STILL:
            if (fabsf(tracker->smoothed_velocity) > VELOCITY_THRESHOLD) {
                tracker->state = tracker->smoothed_velocity > 0 ? AXIS_INCREASING : AXIS_DECREASING;
                memcpy(tracker->start_quat, q_prev, sizeof(float)*4); // Save the start quaternion
                tracker->reversal_counter = 0;
                tracker->possible_peak_angle = 0.0f; // Reset possible peak angle
            }
            break;

        case AXIS_INCREASING:
            if (tracker->smoothed_velocity < VELOCITY_THRESHOLD) {
                tracker->state = AXIS_PEAKING;
                tracker->restore_state = AXIS_INCREASING; // Save the state for restoration
            } else {
                tracker->reversal_counter = 0;
                memcpy(tracker->peak_quat, q_curr, sizeof(float)*4); // Save the peak quaternion
            }
            break;

        case AXIS_DECREASING:
            if (tracker->smoothed_velocity > -VELOCITY_THRESHOLD) {
                tracker->state = AXIS_PEAKING;
                tracker->restore_state = AXIS_DECREASING; // Save the state for restoration
            } else {
                tracker->reversal_counter = 0;
                memcpy(tracker->peak_quat, q_curr, sizeof(float)*4); // Save the peak quaternion
            }
            break;

        case AXIS_PEAKING:
            tracker->prev_peak_angle = tracker->possible_peak_angle;
            tracker->possible_peak_angle = rotation_about_axis(tracker->start_quat, tracker->peak_quat, x_local[which_axis], y_local[which_axis]); // Calculate angle difference
            if(tracker->reversal_counter < STALLLIMIT) {
                tracker->reversal_counter++;
                if(tracker->restore_state == AXIS_INCREASING && tracker->smoothed_velocity > VELOCITY_THRESHOLD) {
                    tracker->state = AXIS_INCREASING; // Continue increasing
                    if(tracker->possible_peak_angle > tracker->prev_peak_angle) {
                        tracker->reversal_counter = 0;
                        memcpy(tracker->peak_quat, q_curr, sizeof(float)*4); // Save the peak quaternion
                    }
                } else if(tracker->restore_state == AXIS_DECREASING && tracker->smoothed_velocity < -VELOCITY_THRESHOLD) {
                    tracker->state = AXIS_DECREASING; // Continue decreasing
                    if(tracker->possible_peak_angle < tracker->prev_peak_angle) {
                        tracker->reversal_counter = 0;
                        memcpy(tracker->peak_quat, q_curr, sizeof(float)*4); // Save the peak quaternion
                    }
                }
            } else {
                if( fabsf(tracker->possible_peak_angle) < DEG2RAD(20.0f) || tracker->possible_peak_angle == DEG2RAD(90.0f)){
                    tracker->state =  AXIS_STILL; // Restore to previous state
                    tracker->reversal_counter = 0; // Reset reversal counter
                    tracker->smoothed_velocity = 0.0f; // Reset smoothed velocity
                    tracker->angle_diff = 0.0f; // Reset angle difference
                    tracker->angle_velocity = 0.0f; // Reset angle velocity
                    tracker->smoothed_velocity = 0.0f; // Reset smoothed velocity
                }else {
                    tracker->angle_diff = tracker->possible_peak_angle; // Update angle difference
                    tracker->state = AXIS_PEAKED; // Set state to peaked
                }
            }
            break;

        case AXIS_PEAKED:
            tracker->state = AXIS_STILL;
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
        case THUMB_EXTENDED: return "Thumb Extended";
        case THUMB_AlIGNED:  return "Thumb Aligned";
        case THUMB_UNDER_PALM:  return "Thumb on Palm";
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

