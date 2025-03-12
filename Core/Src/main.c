/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "UART_LED_Controller.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
UART_LED_Controller uartController;
CAN_TxHeaderTypeDef TxHeader; // Структура TxHeader отвечает за отправку кадров
CAN_RxHeaderTypeDef RxHeader; // Структура RxHeader отвечает за прием кадров
uint8_t TxData[8] = {0,}; // Массив TxData содержит полезные данные для отправки
uint8_t RxData[8] = {0,}; // Массив RxData содержит принятые полезные данные
uint32_t TxMailbox = 0; // Переменная TxMailbox содержит номер корзины для отправки

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void CAN_Filter_Config();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Функция callback для приема сообщения (прерывание)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        // что нибудь делаем когда пришло сообщение
        if(RxHeader.StdId != 0x0378)
        {
          UART_printf(&uartController, "ID %04lX %d %d %d\r\n", RxHeader.StdId, RxData[0], RxData[1], RxData[2]);
        }
        else if(RxHeader.StdId != 0x0126)
        {
          UART_printf(&uartController, "ID %04lX %d %d %d\r\n", RxHeader.StdId, RxData[0], RxData[1], RxData[2]);
        }
    }
}

// Функция callback для ошибок CAN (прерывание)
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t er = HAL_CAN_GetError(hcan);
    UART_printf(&uartController, "Err CAN DEC:%lu HEX:%08lX\r\n", er, er);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  UART_LED_Controller_Init(&uartController, &huart1); // Инициализируем контроллер UART
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  UART_printf(&uartController, "Hello!\r\n");

  // заполняем структуру отвечающую за отправку кадров
  TxHeader.StdId = 0x0378;
  TxHeader.ExtId = 0;
  TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
  TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = 0;
  for(uint8_t i = 0; i < 8; i++)
  {
      TxData[i] = (i + 10);
  }

  CAN_Filter_Config(); // Настраиваем фильтр CAN
  HAL_CAN_Start(&hcan);  // Запускаем контроллер CAN
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF |  CAN_IT_LAST_ERROR_CODE); // Включаем прерывание приема CAN сообщений | ошибок CAN | ошибок шины | последнего кода ошибки

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // отправляем сообщение по CAN
    TxHeader.StdId = 0x0000;
    TxData[0] = 22;

    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);

    if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      UART_printf(&uartController, "Err SEND\r\n");
    }

    HAL_Delay(1500);


    TxHeader.StdId = 0x0001;
    TxData[0] = 77;

    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);

    if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      UART_printf(&uartController, "Err SEND\r\n");
    }

    HAL_Delay(1500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void CAN_Filter_Config()
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;                        // Номер банка фильтра
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;    // Фильтр по маске
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;   // 32-битный фильтр
    sFilterConfig.FilterIdHigh = 0x0000;                 // ID фильтра (старшая часть)
    sFilterConfig.FilterIdLow = 0x0000;                  // ID фильтра (младшая часть)
    sFilterConfig.FilterMaskIdHigh = 0x0000;             // Маска фильтра (старшая часть)
    sFilterConfig.FilterMaskIdLow = 0x0000;              // Маска фильтра (младшая часть)
    sFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;   // Используем FIFO0
    sFilterConfig.FilterActivation = ENABLE;             // Включаем фильтр

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();  // Обработчик ошибки
    }
}

// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//     CAN_RxHeaderTypeDef RxHeader;
//     uint8_t RxData[8];

//     if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
//     {
//         if (RxHeader.IDE == CAN_ID_STD)  // Стандартный идентификатор
//         {
//             if (RxHeader.StdId == CAN_CMD_Test_Send)  // Проверяем команду
//             {
//                 CAN_Send_Ok();
//             }
//             else if (RxHeader.StdId == CAN_CMD_Test_Ok)
//             {
//                 // Можно добавить код обработки, например, мигание светодиодом
//             }
//         }
//     }
// }

// void CAN_Send_Test(void)
// {
//     CAN_TxHeaderTypeDef TxHeader;
//     uint8_t TxData[3] = {0x00, 0x01, 0x02};
//     uint32_t TxMailbox;

//     TxHeader.StdId = 0x00;
//     TxHeader.ExtId = 0x00;     // Не используем расширенный ID
//     TxHeader.IDE = CAN_ID_STD;
//     TxHeader.RTR = CAN_RTR_DATA;
//     TxHeader.DLC = 3;          // Длина блока данных 3 байта
//     TxHeader.TransmitGlobalTime = DISABLE;

//     if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//     {
//         Error_Handler();
//     }
// }


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
