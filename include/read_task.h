#ifndef READ_TASK_H
#define READ_TASK_H

#define USER_BUTTON_GPIO 0  // GPIO pin connected to the user button
#include "espnow_comm.h"

void setup_btn(void);
void read_imu_data_task(void* arg);

#endif
