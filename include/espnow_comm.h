#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

extern QueueHandle_t rx_gesture_queue; // Queue to hold gesture data

void rx_gesture_queue_init(void);

void init_espnow();
void send_gesture_byte(uint8_t gesture);

#endif // ESPNOW_COMM_H
