#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "data_timer.h"
#include "imu_spi.h"

//Timer handles
esp_timer_handle_t data_timer;

TaskHandle_t imu_read_task_handle;

void data_timer_callback(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(imu_read_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void setup_data_timer() {
    const esp_timer_create_args_t timer_args = {
        .callback = &data_timer_callback,
        .arg = NULL,
        .name = "data_timer"
    };

    // Create the periodic timer
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &data_timer));

    // Start the timer with a period of 10,000 microseconds (100Hz)
    ESP_ERROR_CHECK(esp_timer_start_periodic(data_timer, 10000));
}