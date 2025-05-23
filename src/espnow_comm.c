#include "espnow_comm.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include <string.h>
#include <esp_now.h>
#include "nvs_flash.h"

#define GESTURE_QUEUE_SIZE 10 // Size of the gesture queue

//static uint8_t audio_mac[] = {0x74, 0x4d, 0xbd, 0x81, 0x6d, 0x18}; // MAC address of Primary Leader Module (ie. Right Glove)
static uint8_t audio_mac[] = {0x98, 0x3D, 0xAE, 0xE7, 0x9B, 0x60}; // MAC address of uController 2 (ie. Audio Module)
static esp_now_peer_info_t peerInfo;

void rx_gesture_queue_init(void) 
{
    rx_gesture_queue = xQueueCreate(GESTURE_QUEUE_SIZE, sizeof(uint8_t*)); // Create a queue to hold gesture data
    if (rx_gesture_queue == NULL) 
    {
        printf("Failed to create gesture queue\n");
    } else {
        printf("Gesture queue created successfully\n");
    }
}  

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) 
{
    if (len == 1) 
    {
        
        uint8_t gesture = incomingData[0];
        printf("Received gesture: %d\n", gesture);

        // Place gesture into the queue
        xQueueSendToBack(rx_gesture_queue, &gesture, portMAX_DELAY); // Send the gesture to the queue
    }
    else
    {
        printf("Received invalid data length: %d\n", len);
    }
}

void init_espnow() 
{
    rx_gesture_queue_init(); // Initialize the gesture queue

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    //esp_now_init();
    if (esp_now_init() != ESP_OK) 
    {
        printf("ESP-NOW initialization failed\n");
        return;
    }
    //esp_now_register_recv_cb(OnDataRecv);
    esp_err_t err = esp_now_register_recv_cb(OnDataRecv);
    if (err != ESP_OK) 
    {
        printf("Failed to register receive callback: %s\n", esp_err_to_name(err));
        return;
    }
    
    // Register peer
    memcpy(peerInfo.peer_addr, audio_mac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) 
    {
        //ESP_LOGE(TAG, "Failed to add peer");
        printf("Failed to add peer\n");
        return;
    }
}
// This is the function you need to call to send data to the audio module
void send_gesture_byte(uint8_t gesture) 
{
    esp_err_t result = esp_now_send(audio_mac, &gesture, sizeof(gesture));

    if (result == ESP_OK) 
    {
        //ESP_LOGI(TAG, "Sent: 0x%02X", gesture);
        printf("Sent: 0x%02X\n", gesture);
    } else {
        //ESP_LOGE(TAG, "Send error: %d", result);
        printf("Send error: %d\n", result);
    }
}