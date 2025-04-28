#include <math.h>
#include "gesturelibrary.h"

#define DISP_OUT_THRESHOLD_DEG   120.0f   // Minimum degrees swept outward
#define DISP_BACK_THRESHOLD_DEG  120.0f   // Minimum degrees swept back

typedef enum 
{
    DISP_STATE_IDLE,
    DISP_STATE_OUTWARD,
    DISP_STATE_RETURN
} DisperseState;


GestureState disperse(IMUState *imu_state) 
{
    // Persisting state across calls
    static DisperseState disp_state;

    AxisTracker *tracker = &imu_state[BOH].axis[AXIS_SPREAD];

    switch (disp_state) {
        case DISP_STATE_OUTWARD:
            if (tracker->state == AXIS_PEAKED && imu_state[BOH].orientation == IMU_FLAT_UP) 
            {
                // Check if we moved far enough
                if (RAD2DEG(tracker->angle_diff) >= -DISP_OUT_THRESHOLD_DEG) 
                {
                    disp_state = DISP_STATE_RETURN;
                } else {
                    disp_state = DISP_STATE_OUTWARD;
                }
            } else {
                disp_state = DISP_STATE_OUTWARD;
            }
            break;

        case DISP_STATE_RETURN:
            if (tracker->state == AXIS_PEAKED && imu_state[BOH].orientation == IMU_FLAT_UP) 
            {
                if (RAD2DEG(tracker->angle_diff) <= DISP_BACK_THRESHOLD_DEG) 
                {
                    // Gesture complete!
                    send_gesture_byte('A');
                    printf("Disperse gesture completed\n");
                    disp_state = DISP_STATE_OUTWARD;
                    return GESTURE_COMPLETE;
                } else 
                {
                    // Return too small → reset
                    disp_state = DISP_STATE_OUTWARD;
                }
            }
            break;

        default:
            // Reset if we get into an unknown state
            disp_state = DISP_STATE_OUTWARD;
            break;
    }

    return GESTURE_INCOMPLETE;
}
