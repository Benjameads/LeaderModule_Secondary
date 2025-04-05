#ifndef DATA_TIMER_H
#define DATA_TIMER_H

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void setup_data_timer();
void data_timer_callback(void* arg);

//Timer handles
extern esp_timer_handle_t data_timer;
extern TaskHandle_t imu_read_task_handle;

#endif // DATA_TIMER_H
