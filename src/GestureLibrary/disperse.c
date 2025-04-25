#include <math.h>
#include "gesturelibrary.h"

#define DISP_OUT_THRESHOLD_DEG   60.0f   // Minimum degrees swept outward
#define DISP_BACK_THRESHOLD_DEG  60.0f   // Minimum degrees swept back

typedef enum {
    DISP_STATE_IDLE = 0,
    DISP_STATE_OUTWARD,
    DISP_STATE_RETURN
} DisperseState;

// Persisting state across calls
static DisperseState disp_state = DISP_STATE_IDLE;
static float         disp_accum = 0.0f;

GestureState disperse(IMUState *imu_state) {
    // We require palm-down orientation for this gesture
    if (imu_state[BOH].orientation != IMU_FLAT_DOWN) {
        disp_state = DISP_STATE_IDLE;
        disp_accum  = 0.0f;
        return GESTURE_INCOMPLETE;
    }

    AxisTracker *tracker = &imu_state[BOH].axis[AXIS_SPREAD];

    switch (disp_state) {
        case DISP_STATE_IDLE:
            // Detect start of outward motion
            if (tracker->state == AXIS_INCREASING) {
                disp_accum  = tracker->angle_diff;
                disp_state  = DISP_STATE_OUTWARD;
            }
            break;

        case DISP_STATE_OUTWARD:
            // While still increasing, keep summing
            if (tracker->state == AXIS_INCREASING) {
                disp_accum += tracker->angle_diff;
            }
            // Once we hit the peak of the outward sweep...
            else if (tracker->state == AXIS_PEAKED) {
                // Check if we moved far enough
                if (disp_accum >= DISP_OUT_THRESHOLD_DEG) {
                    disp_state = DISP_STATE_RETURN;
                    disp_accum = 0.0f;
                } else {
                    // Not far enough → reset
                    disp_state = DISP_STATE_IDLE;
                    disp_accum = 0.0f;
                }
            }
            break;

        case DISP_STATE_RETURN:
            // Accumulate the return (decreasing) motion
            if (tracker->state == AXIS_DECREASING) {
                disp_accum += fabsf(tracker->angle_diff);
            }
            // On the peak of the return motion...
            else if (tracker->state == AXIS_PEAKED) {
                if (disp_accum >= DISP_BACK_THRESHOLD_DEG) {
                    // Gesture complete!
                    send_gesture_byte('A');
                    disp_state = DISP_STATE_IDLE;
                    disp_accum = 0.0f;
                    return GESTURE_COMPLETE;
                } else {
                    // Return too small → reset
                    disp_state = DISP_STATE_IDLE;
                    disp_accum = 0.0f;
                }
            }
            break;
    }

    return GESTURE_INCOMPLETE;
}
