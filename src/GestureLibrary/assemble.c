#include <math.h>
#include "gesturelibrary.h"

#define CURL_THRESH   20.0f   // Minimum degrees swept outward
#define SPREAD_THRESH 20.0f   // Minimum degrees swept outward

typedef enum {
    STATE_IDLE,
    STATE_STAGE1,
    STATE_STAGE2
} State;


GestureState assemble(IMUState *imu_state) {
    // Persisting state across calls
    static State state;
    static int spread_counter = 0; // Counter for spread gesture
    static int curl_counter = 0; // Counter for curl gesture

    AxisTracker *tracker_curl = &imu_state[BOH].axis[AXIS_CURL];
    AxisTracker *tracker_spread = &imu_state[BOH].axis[AXIS_SPREAD];

    if(imu_state[THUMB].orientation != THUMB_AlIGNED) {
        state = STATE_STAGE1; // Reset to stage 1 if thumb is not aligned
    }

    switch (state) {
        //stage 1: hand moves from flat imu down in front of body to palm infront of face
        case STATE_STAGE1:
            if (tracker_curl->state == AXIS_PEAKED && imu_state[BOH].orientation == IMU_BACKWARD) {
                if (fabsf(RAD2DEG(tracker_curl->angle_diff)) >= CURL_THRESH) {
                    state = STATE_STAGE2;
                    curl_counter++; // Increment curl counter
                } else {
                    state = STATE_STAGE1;
                    spread_counter = 0; // Reset spread counter
                    curl_counter = 0; // Reset curl counter
                }
            } else {
                state = STATE_STAGE1;
            }
            break;

        case STATE_STAGE2:
            if (tracker_spread->state == AXIS_PEAKED && imu_state[BOH].orientation == IMU_BACKWARD) {
                if (fabsf(RAD2DEG(tracker_spread->angle_diff)) >= SPREAD_THRESH) {
                    spread_counter++; // Increment spread counter
                    if(spread_counter >= 2 && curl_counter >= 2) {
                        // Gesture complete!
                        send_gesture_byte('B'); // Send the gesture to the audio module
                        printf("Assemble gesture completed\n");
                        state = STATE_STAGE1; // Reset state after gesture completion
                        return GESTURE_COMPLETE;
                        spread_counter = 0; // Reset spread counter
                        curl_counter = 0; // Reset curl counter
                    } else {
                        state = STATE_STAGE2; // Stay in stage 2 if not enough curls/spreads
                    }
                } else {
                    // Return too small → reset
                    state = STATE_STAGE1;
                    spread_counter = 0; // Reset spread counter
                    curl_counter = 0; // Reset curl counter
                }
            }
            else {
                state = STATE_STAGE2; // Stay in stage 2 if not enough curls/spreads
            }
            break;

        default:
            // Reset if we get into an unknown state
            state = STATE_STAGE1;
            break;
    }

    return GESTURE_INCOMPLETE;
}
