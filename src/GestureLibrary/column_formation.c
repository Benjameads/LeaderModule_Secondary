#include <math.h>
#include "gesturelibrary.h"

#define SPREAD_THRESH   20.0f   // Minimum degrees swept outward

typedef enum 
{
    STATE_IDLE,
    STATE_STAGE1,
    STATE_STAGE2
} State;


GestureState column_formation(IMUState *imu_state) 
{
    // Persisting state across calls
    static State state;

    AxisTracker *tracker = &imu_state[BOH].axis[AXIS_SPREAD];

    switch (state) {
        //stage 1: hand moves from flat imu down in front of body to palm infront of face
        case STATE_STAGE1:
            if (tracker->state == AXIS_PEAKED && imu_state[BOH].orientation == IMU_LEFT) 
            {
                if (RAD2DEG(tracker->angle_diff) <= -SPREAD_THRESH) {
                    state = STATE_STAGE2;
                } else {
                    state = STATE_STAGE1;
                }
            } else {
                state = STATE_STAGE1;
            }
            break;

        case STATE_STAGE2:
            if (tracker->state == AXIS_PEAKED && imu_state[BOH].orientation == IMU_BACKWARD) 
            {
                if (RAD2DEG(tracker->angle_diff) >= SPREAD_THRESH) 
                {
                    // Gesture complete!
                    send_gesture_byte('J');
                    printf("Collumn Formation gesture completed\n");
                    state = STATE_STAGE1;
                    return GESTURE_COMPLETE;
                } else 
                {
                    // Return too small → reset
                    state = STATE_STAGE1;
                }
            }
            break;

        default:
            // Reset if we get into an unknown state
            state = STATE_STAGE1;
            break;
    }

    return GESTURE_INCOMPLETE;
}