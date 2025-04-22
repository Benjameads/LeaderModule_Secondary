// #include "gesturelibrary.h"
// #include "orientation_task.h"
// #include "imu_read.h"


// // This will use statemachine style logic, where based on the current state and orientation data
// // the next state is determined. The state machine will be reset to the start state when the gesture is complete or cancelled.

// GestureState disperse(GestureOrientationData* gesture_data){
//     // Disperse is a hand signal telling group members to disperse
//     // The gesture beginns with the hand flat next to opposite shoulder and palm facing the ground
//     // the hand will then extend away from the body and stop with arm in line with the shoulders and palm facing the ground

//     static GestureState state = GESTURE_START, restorestate; // Initialize the state variable to GESTURE_NO_PROGRESS
//     struct OrientationData start_data[NUMBER_OF_IMUS] = {0};
//     float yawdif[NUMBER_OF_IMUS] = {0};
//     //float pitchdif[NUMBER_OF_IMUS] = {0};
//     //float rolldif[NUMBER_OF_IMUS] = {0};
//     static uint8_t stallcount = 0; // Initialize stall count to 0

//     for(int i = 0; i < NUMBER_OF_IMUS; i++){
//         yawdif[i] = YAWDIF(i);      // Use macro to get yaw difference
//         //pitchdif[i] = PITCHDIF(i);  // Use macro to get pitch difference
//         //rolldif[i] = ROLLDIF(i);    // Use macro to get roll difference
//     }

//     switch(state) {
//         case GESTURE_START:
//             // Check if the hand is in the starting position (flat next to opposite shoulder)
//             if (IS_FLAT(BOH)) {
//                 for(int i = 0; i < NUMBER_OF_IMUS; i++){
//                     // Store the starting position of the hand
//                     start_data[i].yaw = gesture_data[i].current.yaw;
//                     start_data[i].pitch = gesture_data[i].current.pitch;
//                     start_data[i].roll = gesture_data[i].current.roll;
//                 }
//                 // Check if the hand is moving away from the body (pitch and roll are increasing)
//                 state = GESTURE_STEP_1; // Move to the no progress state
//             }
//             else {
//                 state = GESTURE_START; // Stay in the start state
//             }
//             break;

//         case GESTURE_STEP_1:
//             // Check if the hand is extending away from the body
//             if (IS_FLAT(BOH)) {
//                 if((STARTYAWDIF(BOH) >= 175) && (yawdif[BOH] < 0)){
//                     // Check if the hand is moving away from the body (Yaw is increasing, pitch and roll should stay around 0)
//                     state = GESTURE_STEP_2; // Move to the next state
//                 }
//                 else if(yawdif[BOH] > 0){
//                     state = GESTURE_STEP_1; // Stay in the step 1 state
//                 }
//                 else if (yawdif[BOH] <= 0){
//                     // If the hand is moving back to start state before the gesture is complete use the stall state to "debounce" the movement
//                     restorestate = state; // Store the current state to restore later
//                     state = GESTURE_STALL; // Move to the stall state
//                 }
//             }
//             else if(!IS_FLAT(BOH)){
//                 //If the hand is not flat next to the opposite shoulder, reset the state to start
//                 //reset logic
//                 state = GESTURE_START; // Move to the start state
//             }
//             break;

//         case GESTURE_STEP_2:
//             // Check if the hand is in the final position (flat next to opposite shoulder)
//             if (IS_FLAT(BOH)) {
//                 if( (fabsf(STARTYAWDIF(BOH))) <= 5){
//                     // Check if the hand within 5 degrees of the starting yaw position (flat next to opposite shoulder)
//                     state = GESTURE_COMPLETE; // Move to the complete state
//                 }
//                 else if(yawdif[BOH] < 0){
//                     state = GESTURE_STEP_2; // Stay in the step 2 state
//                 }
//                 else if (yawdif[BOH] >= 0){
//                     // If the hand is moving back to start state before the gesture is complete use the stall state to "debounce" the movement
//                     restorestate = state; // Store the current state to restore later
//                     state = GESTURE_STALL; // Move to the stall state
//                 }
//             }
//             else if(!IS_FLAT(BOH)){
//                 //If the hand is not flat next to the opposite shoulder, reset the state to start
//                 //reset logic
//                 state = GESTURE_START; // Move to the start state
//             }
//             break;

//         case GESTURE_STALL:
//             //Check restore state to determine debounce logic
//             if (restorestate == GESTURE_STEP_1) {
//                 if(IS_FLAT(BOH) && (yawdif[BOH] >= 0)){
//                     // If the hand is flat next to the opposite shoulder, reset the state to step 1
//                     state = restorestate; // Move to the step 1 state
//                     stallcount = 0; // Reset stall count
//                 }
//                 else if (IS_FLAT(BOH) && (yawdif[BOH] < 0) && (stallcount <= STALLLIMIT)){
//                     stallcount++; // Increment stall count
//                     state = GESTURE_STALL; // Stay in the stall state
//                 }
//                 else{
//                     // If the hand is not flat next to the opposite shoulder, reset the state to start
//                     state = GESTURE_START; // Move to the start state
//                     stallcount = 0; // Reset stall count
//                     printf("Disperse gesture cancelled!\n"); // Print message indicating gesture cancellation
//                 }
//             }
//             else if (restorestate == GESTURE_STEP_2) {
//                 if(IS_FLAT(BOH) && (yawdif[BOH] < 0)){
//                     // If the hand is flat next to the opposite shoulder, reset the state to step 2
//                     state = restorestate; // Move to the step 2 state
//                     stallcount = 0; // Reset stall count
//                 }
//                 else if (IS_FLAT(BOH) && (yawdif[BOH] >= 0) && (stallcount <= STALLLIMIT)){
//                     state = GESTURE_STALL; // Stay in the stall state
//                 }
//                 else{
//                     // If the hand is not flat next to the opposite shoulder, reset the state to start
//                     state = GESTURE_START; // Move to the start state
//                     stallcount = 0; // Reset stall count
//                     printf("Disperse gesture cancelled!\n"); // Print message indicating gesture cancellation
//                 }
//             }
//             break;

//         case GESTURE_COMPLETE:
//             //send disperse command to Secodary MCU
//             //**********************ESPNOW STUFF*********************/
//             printf("Disperse gesture detected!\n");
//             // Reset the state for the next gesture
//             state = GESTURE_START;
//             break;
//     }

//     // Return the current state of the gesture
//     return state;
// }
