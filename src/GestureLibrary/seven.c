#include "gesturelibrary.h"


GestureState seven(IMUState* imu_state)
{
    if ((imu_state[THUMB].orientation != THUMB_EXTENDED) &&
        (imu_state[MIDDLE].orientation == FINGER_STRAIGHT) &&
        (imu_state[INDEX].orientation == FINGER_STRAIGHT) &&
        (imu_state[RING].orientation == FINGER_CURLED) &&
        (imu_state[PINKY].orientation == FINGER_CURLED) &&
        (imu_state[BOH].orientation == IMU_RIGHT))
    {
        send_gesture_byte('O'); // Send the gesture to the audio module //ascii value of 'F' is 70
        printf("Seven gesture detected!\n"); // Print message indicating gesture detection
        return GESTURE_COMPLETE;
    } 
    else 
    {
        return GESTURE_INCOMPLETE;
    }
}