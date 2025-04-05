#ifndef UART_H
#define UART_H

#include "driver/uart.h"
#include "esp_log.h"

#define UART_PORT UART_NUM_0
#define BUF_SIZE 128

void init_uart_for_input();

#endif // UART_H