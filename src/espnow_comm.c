#include "espnow_comm.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include <string.h>
#include "nvs_flash.h"
#include "esp_netif.h"

static const char *TAG = "ESP_NOW_COMM";
static uint8_t peer_mac[] = ESPNOW_PEER_MAC_ADDR;
static espnow_receive_callback_t user_recv_callback = NULL;

// Internal receive callback
// Just logs the recieved data and if it was valid length
static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (len == 1 && user_recv_callback) {
        user_recv_callback(incomingData[0]);
    } else {
        ESP_LOGW(TAG, "Received invalid data length: %d", len);
    }
}

// Internal send status callback
// Just logs whether the send was successful or not
static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}
//When you call espnow_init, you pass in the name of the function you want to call when data is received
esp_err_t espnow_init(espnow_receive_callback_t callback) {
    user_recv_callback = callback;

    //1. Initialize NVS (required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    //2. Initialize TCP/IP stack and event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    //3. Initialize Wi-Fi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    //4. Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());

    //5. Register callbacks
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    //6. Register peer (the other glove)
    esp_now_peer_info_t peerInfo = {
        .channel = 0,
        .encrypt = false
    };
    memcpy(peerInfo.peer_addr, peer_mac, 6);

    if (!esp_now_is_peer_exist(peer_mac)) {
        ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    }

    ESP_LOGI(TAG, "ESP-NOW initialized and peer registered");
    return ESP_OK;
}

esp_err_t espnow_send_byte(uint8_t data) {
    return esp_now_send(peer_mac, &data, sizeof(data));
}
