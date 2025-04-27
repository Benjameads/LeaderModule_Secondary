#include "gesturelibrary.h"


GestureState freeze(IMUState* imu_state)
{
    if ((imu_state[THUMB].orientation == THUMB_AlIGNED) &&
        (imu_state[MIDDLE].orientation == FINGER_CURLED) &&
        (imu_state[INDEX].orientation == FINGER_CURLED) &&
        (imu_state[RING].orientation == FINGER_CURLED) &&
        (imu_state[PINKY].orientation == FINGER_CURLED) &&
        (imu_state[BOH].orientation == IMU_BACKWARD) &&
        (imu_state[BOH].axis[ROLL].angle_diff < 10))
    {
        send_gesture_byte('b'); // Send the gesture to the audio module //ascii value of 'F' is 70
        printf("Freeze gesture detected!\n"); // Print message indicating gesture detection
        return GESTURE_COMPLETE;
    } 
    else 
    {
        return GESTURE_INCOMPLETE;
    }
}