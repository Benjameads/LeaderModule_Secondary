#include "espnow_comm.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "ESP-NOW";
static uint8_t receiver_mac[] = {0x74, 0x4D, 0xBD, 0x81, 0x6D, 0x18};  // MAC Address of Primary Leader Module (right glove)
//static uint8_t receiver_mac[] = {0x98, 0x3D, 0xAE, 0xE7, 0xA7, 0x98};  // MAC Address of uController 1

static esp_now_peer_info_t peerInfo;

// static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//     ESP_LOGI(TAG, "Last Packet Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
// }

void init_espnow() {
    //nvs_flash_init(); //required for wifi to work
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    // Initialize WiFi in STA mode
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error initializing ESP-NOW");
        return;
    }

    //esp_now_register_send_cb(OnDataSent);

    // Register peer
    memcpy(peerInfo.peer_addr, receiver_mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer");
        return;
    }
}

//This is the function that sends the gesture byte to the primary glove
//You call it whenever you recognize a gesture.
void send_gesture_byte(uint8_t gesture) {
    esp_err_t result = esp_now_send(receiver_mac, &gesture, sizeof(gesture));

    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Sent: 0x%02X", gesture);
    } else {
        ESP_LOGE(TAG, "Send error: %d", result);
    }
}

