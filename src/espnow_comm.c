#include "espnow_comm_secondary.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include <string.h>
#include "nvs_flash.h"
#include "esp_netif.h"

static const char *TAG = "ESP_NOW_SENDER";
static uint8_t peer_mac[] = ESPNOW_PEER_MAC_ADDR;

static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

esp_err_t espnow_init_sender_only(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    esp_now_peer_info_t peerInfo = {
        .channel = 0,
        .encrypt = false
    };
    memcpy(peerInfo.peer_addr, peer_mac, 6);

    if (!esp_now_is_peer_exist(peer_mac)) {
        ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    }

    ESP_LOGI(TAG, "ESP-NOW Sender Initialized");
    return ESP_OK;
}

esp_err_t espnow_send_byte(uint8_t data) {
    return esp_now_send(peer_mac, &data, sizeof(data));
}