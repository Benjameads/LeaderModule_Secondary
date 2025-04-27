#include "gesturelibrary.h"


GestureState wedge_formation(IMUState* imu_state)
{
    if ((imu_state[THUMB].orientation != THUMB_UNDER_PALM) &&
        (imu_state[MIDDLE].orientation == FINGER_CURLED) &&
        (imu_state[INDEX].orientation == FINGER_STRAIGHT) &&
        (imu_state[RING].orientation == FINGER_CURLED) &&
        (imu_state[PINKY].orientation == FINGER_STRAIGHT) &&
        (imu_state[BOH].orientation == IMU_FORWARD))
    {
        send_gesture_byte('D'); // Send the gesture to the audio module //ascii value of 'F' is 70
        printf("Wedge Formation gesture detected!\n"); // Print message indicating gesture detection
        return GESTURE_COMPLETE;
    } 
    else 
    {
        return GESTURE_INCOMPLETE;
    }
}