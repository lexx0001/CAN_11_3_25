#include "CANbus.h" 
#include <stm32f1xx_hal_can.h>


/**
 * @brief Отправляет CAN-сообщение
 * @param hcan Указатель на CAN-структуру
 * @param stdId Идентификатор стандартного кадра (StdId)
 * @param data Указатель на массив данных (максимум 8 байт)
 * @param length Длина данных (1-8 байт)
 * @return HAL_StatusTypeDef (HAL_OK при успешной отправке, иначе ошибка)
 */

 HAL_StatusTypeDef CAN_SendMessage(CAN_HandleTypeDef *hcan, uint16_t Id, uint8_t *data, uint8_t length) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    if (length > 8) {
        return HAL_ERROR; // Длина данных не корректна
    }

    // Заполняем структуру заголовка CAN-сообщения
    TxHeader.StdId = 0;
    TxHeader.ExtId = Id;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.DLC = length;
    TxHeader.TransmitGlobalTime = DISABLE;

    // Ждем, пока освободится хотя бы одна почтовая ячейка
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);

    // Отправляем сообщение
    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox) != HAL_OK) {
        return HAL_ERROR; // Ошибка отправки
    }

    return HAL_OK; // Успешная отправка
}




// /**
//  * @brief Получает CAN-сообщение из FIFO0
//  * @param hcan Указатель на CAN-структуру
//  * @param stdId Указатель для сохранения StdId сообщения
//  * @param dlc Указатель для сохранения длины данных (DLC)
//  * @param data Указатель на массив для сохранения данных (макс. 8 байт)
//  * @return HAL_StatusTypeDef (HAL_OK при успешном приеме, иначе ошибка)
//  */
// HAL_StatusTypeDef CAN_ReceiveMessage(CAN_HandleTypeDef *hcan, uint16_t *stdId, uint8_t *dlc, uint8_t *data) {
//     CAN_RxHeaderTypeDef RxHeader;
//     uint8_t RxData[8];

//     // Получаем сообщение из FIFO0
//     if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
//         return HAL_ERROR; // Ошибка приема
//     }

//     // Сохраняем данные в переданных переменных
//     *stdId = RxHeader.StdId;
//     *dlc = RxHeader.DLC;
//     memcpy(data, RxData, RxHeader.DLC);

//     return HAL_OK;
// }
