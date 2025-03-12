#include "UART_LED_Controller.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// Инициализация структуры
void UART_LED_Controller_Init(UART_LED_Controller* controller, UART_HandleTypeDef* huart) {
    controller->huart = huart;
}

// Отправка строки через UART
void UART_LED_Controller_Send(UART_LED_Controller* controller, const char* data) {
    uint16_t size = strlen(data);
    HAL_UART_Transmit(controller->huart, (uint8_t*)data, size, UART_TIMEOUT);
}

// Отправка числа через UART
void UART_LED_Controller_SendNumber(UART_LED_Controller* controller, int number) {
    char buffer[UART_BUFFER_SIZE];
    snprintf(buffer, UART_BUFFER_SIZE, "%d\r\n", number);
    UART_LED_Controller_Send(controller, buffer);
}

// Отправка форматированной строки через UART
void UART_printf(UART_LED_Controller* controller, const char* format, ...) {
    char buffer[UART_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, UART_BUFFER_SIZE, format, args);
    va_end(args);

    UART_LED_Controller_Send(controller, buffer);
}
