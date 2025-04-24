#include "gesturelibrary.h"


GestureState eight(IMUState* imu_state)
{
    if ((imu_state[THUMB].orientation == FINGER_CURLED) &&
        (imu_state[MIDDLE].orientation == FINGER_STRAIGHT) &&
        (imu_state[INDEX].orientation == FINGER_STRAIGHT) &&
        (imu_state[RING].orientation == FINGER_STRAIGHT) &&
        (imu_state[PINKY].orientation == FINGER_CURLED) &&
        (imu_state[BOH].orientation == IMU_RIGHT))
    {
        send_gesture_byte('p'); // Send the gesture to the audio module //ascii value of 'F' is 70
        printf("Eight gesture detected!\n"); // Print message indicating gesture detection
        return GESTURE_COMPLETE;
    } 
    else 
    {
        return GESTURE_INCOMPLETE;
    }
}