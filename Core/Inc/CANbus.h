#ifndef CANBUS_H
#define CANBUS_H

#include <stm32f1xx_hal.h>

// ...existing code...

HAL_StatusTypeDef CAN_SendMessage(CAN_HandleTypeDef *hcan, uint16_t stdId, uint8_t *data, uint8_t length);
// HAL_StatusTypeDef CAN_ReceiveMessage(CAN_HandleTypeDef *hcan, uint16_t *stdId, uint8_t *dlc, uint8_t *data);

// ...existing code...

#endif // CANBUS_H
