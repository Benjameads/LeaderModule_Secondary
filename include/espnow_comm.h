#ifndef ESPNOW_COMM_SECONDARY_H
#define ESPNOW_COMM_SECONDARY_H

#include <stdint.h>
#include "esp_err.h"

// Replace with the MAC address of the Primary Glove
#define ESPNOW_PEER_MAC_ADDR {0x98, 0x3D, 0xAE, 0xE7, 0xA7, 0x98}

// Initializes ESP-NOW for sending only (no receive callback)
esp_err_t espnow_init_sender_only(void);

// Send a single byte of data
esp_err_t espnow_send_byte(uint8_t data);

#endif // ESPNOW_COMM_SECONDARY_H
