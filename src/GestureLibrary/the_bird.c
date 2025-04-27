#include "gesturelibrary.h"


GestureState the_bird(IMUState* imu_state)
{
    if ((imu_state[MIDDLE].orientation == FINGER_STRAIGHT) &&
        (imu_state[INDEX].orientation == FINGER_CURLED) &&
        (imu_state[RING].orientation == FINGER_CURLED) &&
        (imu_state[PINKY].orientation == FINGER_CURLED) &&
        (imu_state[BOH].orientation == IMU_BACKWARD))
    {
        send_gesture_byte('F'); // Send the gesture to the audio module //ascii value of 'F' is 70
        printf("The Bird gesture detected!\n"); // Print message indicating gesture detection
        return GESTURE_COMPLETE;
    } 
    else 
    {
        return GESTURE_INCOMPLETE;
    }
}