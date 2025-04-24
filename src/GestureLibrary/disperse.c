// #include "gesturelibrary.h"
// #include "orientation_task.h"
// #include "imu_read.h"


// This will use statemachine style logic, where based on the current state and orientation data
// the next state is determined. The state machine will be reset to the start state when the gesture is complete or cancelled.

// GestureState disperse(IMUState* imu_data)
// {
//     if (imu_data[BOH].axis[AXIS_SPREAD].state == AXIS_PEAKED)
//     {
//         // Check if the gesture is complete
//         if (imu_data[BOH].axis[AXIS_SPREAD].angle_diff > 30.0f) // Example threshold for completion
//         {
//             send_gesture_byte('d'); // Send the gesture to the audio module
//             printf("Disperse gesture detected!\n"); // Print message indicating gesture detection
//             return GESTURE_COMPLETE;
//         }
//     }
    // switch(state) {
    //     case GESTURE_START:
            
    //         break;

    //     case GESTURE_STEP_1:
            
    //         break;

    //     case GESTURE_STEP_2:
    //         break;

    //     case GESTURE_STALL:
           
    //         break;

    //     case GESTURE_COMPLETE:
            
    // }

    // Return the current state of the gesture
//     return state;
// }
