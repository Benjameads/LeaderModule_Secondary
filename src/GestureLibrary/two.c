#include "gesturelibrary.h"


GestureState two(IMUState* imu_state)
{
    if ((imu_state[THUMB].orientation == THUMB_UNDER_PALM) &&
        (imu_state[MIDDLE].orientation == FINGER_STRAIGHT) &&
        (imu_state[INDEX].orientation == FINGER_STRAIGHT) &&
        (imu_state[RING].orientation == FINGER_CURLED) &&
        (imu_state[PINKY].orientation == FINGER_CURLED) &&
        (imu_state[BOH].orientation == IMU_BACKWARD)  &&
        (imu_state[BOH].axis[AXIS_TWIST].state == AXIS_STILL) && 
        (imu_state[BOH].axis[AXIS_SPREAD].state == AXIS_STILL) &&
        (imu_state[BOH].axis[AXIS_CURL].state == AXIS_STILL))
    {
        send_gesture_byte('j'); // Send the gesture to the audio module //ascii value of 'F' is 70
        printf("Two gesture detected!\n"); // Print message indicating gesture detection
        return GESTURE_COMPLETE;
    } 
    else 
    {
        return GESTURE_INCOMPLETE;
    }
}