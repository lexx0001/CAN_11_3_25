#ifndef UART_LED_CONTROLLER_H
#define UART_LED_CONTROLLER_H

#include "main.h"
#include <stdbool.h>
#include <stm32f1xx_hal.h>  // Убедитесь, что этот файл доступен в вашем проекте

// Размер общего буфера для форматирования
#define UART_BUFFER_SIZE 128
#define UART_TIMEOUT 100  // Таймаут для передачи UART (мс)

// Структура для хранения конфигурации
typedef struct {
    UART_HandleTypeDef* huart;
} UART_LED_Controller;

// Функции управления
void UART_LED_Controller_Init(UART_LED_Controller* controller, UART_HandleTypeDef* huart);
void UART_LED_Controller_Send(UART_LED_Controller* controller, const char* data);
void UART_LED_Controller_SendNumber(UART_LED_Controller* controller, int number);
void UART_printf(UART_LED_Controller* controller, const char* format, ...);

#endif // UART_LED_CONTROLLER_H
