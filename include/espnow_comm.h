// #ifndef ESPNOW_COMM_H
// #define ESPNOW_COMM_H

// #include <stdint.h>
// #include "esp_err.h"

// // Replace with the MAC address of the paired glove (Secondary or Primary)
// #define ESPNOW_PEER_MAC_ADDR {0x98, 0x3D, 0xAE, 0xE7, 0xA7, 0x98}

// // Function pointer type for user-defined receive handler
// typedef void (*espnow_receive_callback_t)(uint8_t data);

// // Initialization: sets up ESP-NOW and registers callbacks
// esp_err_t espnow_init(espnow_receive_callback_t callback);

// // Send a single byte of data
// esp_err_t espnow_send_byte(uint8_t data);

// #endif // ESPNOW_COMM_H

#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <stdint.h>

void init_espnow();
void send_to_audio_module(uint8_t gesture);

#endif // ESPNOW_COMM_H
