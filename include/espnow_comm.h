// #ifndef ESPNOW_COMM_SECONDARY_H
// #define ESPNOW_COMM_SECONDARY_H

// #include <stdint.h>
// #include "esp_err.h"

// // Replace with the MAC address of the Primary Glove
// #define ESPNOW_PEER_MAC_ADDR {0x74, 0x4D, 0xBD, 0x81, 0x6D, 0x18}

// // Initializes ESP-NOW for sending only (no receive callback)
// esp_err_t espnow_init_sender_only(void);

// // Send a single byte of data
// esp_err_t espnow_send_byte(uint8_t data);

// #endif // ESPNOW_COMM_SECONDARY_H

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
