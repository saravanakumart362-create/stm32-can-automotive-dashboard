/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Trame échangée avec le dashboard PC par UART. */
typedef struct
{
    uint8_t cmd;
    uint8_t payload[4];
} FrameCom;

/* États de la machine à états de réception UART (voir HAL_UART_RxCpltCallback). */
typedef enum
{
    WAIT_FOR_SOF,
    WAIT_FOR_CMD,
    WAIT_FOR_DATA1,
    WAIT_FOR_DATA2,
    WAIT_FOR_DATA3,
    WAIT_FOR_DATA4
} FrameStateTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ==================== SRS-SYS-02 ====================
 * Exigence SRS : firmware modulaire (can_node.c/h, uart_bridge.c/h, can_config.h).
 * Statut : NON FAIT — tout en un seul main.c, reporté en V2 faute de temps. */

/* ==================== SRS-UART-02 ====================
 * Trame UART fixe 6 octets : SOF + CMD + PAYLOAD(4).
 * Aucun checksum (conforme au SRS d'origine). */
#define SOF_PATTERN  0xA5U

#define CMD_LED_ON          0x01U
#define CMD_LED_OFF         0x02U
#define CMD_READ_SENSOR     0x10U
#define CMD_TRIGGER_ABS     0x11U
#define CMD_SENSOR_VALUE    0x20U
#define CMD_ACK             0x50U

#define CMD_SPEED_MOTOR       0x21U
#define CMD_TEMPERATURE       0x22U
#define CMD_DEBIT_FLUIDE      0x23U
#define CMD_BATTERY_VOLTAGE   0x24U
#define CMD_EXTERNAL_LIGHT    0x25U
#define CMD_WINDOW_LEFT       0x31U
#define CMD_WINDOW_RIGHT      0x32U
#define CMD_PORTE_DROITE      0x33U

/* ==================== SRS-CAN-02 ====================
 * Table d'allocation des IDs CAN (5 nœuds sur le même bus).
 * Nœud    | Signal          | ID CAN | CMD UART
 * Amine   | Pression        | 0x55C  | CMD_SENSOR_VALUE
 * partagé | ABS trigger/ack | 0x679  | CMD_ACK
 * Yassine | Speed Motor     | 0x406  | CMD_SPEED_MOTOR
 * Souha   | Temperature     | 0x456  | CMD_TEMPERATURE
 * Eya     | Debit Fluide    | 0x390  | CMD_DEBIT_FLUIDE
 * Yassine | Window Left     | 0x401  | CMD_WINDOW_LEFT
 * Souha   | Window Right    | 0x402  | CMD_WINDOW_RIGHT
 * Eya     | Porte Droite    | 0x790  | CMD_PORTE_DROITE
 * Nour    | Battery Voltage | 0x222  | CMD_BATTERY_VOLTAGE
 * Nour    | External Light  | 0x221  | CMD_EXTERNAL_LIGHT */
#define CANID_PRESSURE          0x55CU
#define CANID_ABS               0x679U
#define CANID_SPEED_MOTOR       0x406U
#define CANID_TEMPERATURE       0x456U
#define CANID_DEBIT_FLUIDE      0x390U
#define CANID_WINDOW_LEFT       0x401U
#define CANID_WINDOW_RIGHT      0x402U
#define CANID_PORTE_DROITE      0x790U
#define CANID_BATTERY_VOLTAGE   0x222U
#define CANID_EXTERNAL_LIGHT    0x221U

/* Mettre à 0 pour couper la génération de pression de test. */
#define ENABLE_LOCAL_PRESSURE_DEMO   1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

uint8_t rx_byte;
FrameCom RxFrame;
static FrameStateTypeDef state = WAIT_FOR_SOF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void NewFrameReceivedCallback(FrameCom *rxFrame);
void SendFrame(uint8_t cmd, uint8_t *payload);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* ==================== SRS-CAN-01 ====================
   * Filtre CAN passe-tout (mask=0x0000) : écoute les 10 IDs des 5 nœuds
   * sur un seul filtre. */
  CAN_FilterTypeDef filter;
  filter.FilterBank           = 0;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh         = 0x0000;
  filter.FilterIdLow          = 0x0000;
  filter.FilterMaskIdHigh     = 0x0000;
  filter.FilterMaskIdLow      = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation     = ENABLE;
  filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &filter);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

  TxHeader.StdId              = CANID_PRESSURE;
  TxHeader.ExtId              = 0x00;
  TxHeader.IDE                = CAN_ID_STD;
  TxHeader.RTR                = CAN_RTR_DATA;
  TxHeader.DLC                = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
    HAL_Delay(500);

#if ENABLE_LOCAL_PRESSURE_DEMO
    /* Génère une valeur de pression croissante pour tester
     * la chaîne CAN -> UART -> dashboard. */
    static uint8_t pressure = 0;
    pressure++;
    TxData[0] = 0x00;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = pressure;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;

    FrameCom TxFrame;
    TxFrame.cmd = CMD_SENSOR_VALUE;
    TxFrame.payload[0] = 0x00;
    TxFrame.payload[1] = 0x00;
    TxFrame.payload[2] = 0x00;
    TxFrame.payload[3] = pressure;
    SendFrame(TxFrame.cmd, TxFrame.payload);

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        Error_Handler();
    }
#endif /* ENABLE_LOCAL_PRESSURE_DEMO */
  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL; /* nécessaire : multi-nœuds -> ACK bit entre nœuds */
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /* ==================== SRS-SYS-01 ====================
   * Bitrate CAN = 250 kbps (Prescaler=4, BS1=6TQ, BS2=1TQ, APB1=8MHz HSE bypass). */
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ==================== SRS-CAN-03 ====================
 * Relaie chaque trame CAN reçue vers le dashboard PC via UART. */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

    switch (RxHeader.StdId)
    {
        case CANID_PRESSURE:        /* Amine */
        {
            uint8_t payload[4] = {RxData[0], RxData[1], RxData[2], RxData[3]};
            SendFrame(CMD_SENSOR_VALUE, payload);
            break;
        }

        case CANID_ABS:              /* partagé */
        {
            uint8_t payload[4] = {0x01, 0x00, 0x00, 0x00};
            SendFrame(CMD_ACK, payload);
            break;
        }

        case CANID_SPEED_MOTOR:      /* Yassine */
        {
            uint8_t payload[4] = {0x00, 0x00, 0x00, RxData[0]};
            SendFrame(CMD_SPEED_MOTOR, payload);
            break;
        }

        case CANID_TEMPERATURE:      /* Souha */
        {
            uint8_t payload[4] = {RxData[0], RxData[1], RxData[2], RxData[3]};
            SendFrame(CMD_TEMPERATURE, payload);
            break;
        }

        case CANID_DEBIT_FLUIDE:     /* Eya */
        {
            uint8_t payload[4] = {0x00, 0x00, 0x00, RxData[0]};
            SendFrame(CMD_DEBIT_FLUIDE, payload);
            break;
        }

        case CANID_WINDOW_LEFT:      /* Yassine */
        {
            uint8_t payload[4] = {RxData[0], RxData[1], RxData[2], RxData[3]};
            SendFrame(CMD_WINDOW_LEFT, payload);
            break;
        }

        case CANID_WINDOW_RIGHT:     /* Souha */
        {
            uint8_t payload[4] = {RxData[0], RxData[1], RxData[2], RxData[3]};
            SendFrame(CMD_WINDOW_RIGHT, payload);
            break;
        }

        case CANID_PORTE_DROITE:     /* Eya */
        {
            uint8_t payload[4] = {RxData[0], RxData[1], RxData[2], RxData[3]};
            SendFrame(CMD_PORTE_DROITE, payload);
            break;
        }

        case CANID_BATTERY_VOLTAGE:  /* Nour */
        {
            uint8_t payload[4] = {0x00, 0x00, RxData[0], RxData[1]};
            SendFrame(CMD_BATTERY_VOLTAGE, payload);
            break;
        }

        case CANID_EXTERNAL_LIGHT:   /* Nour */
        {
            uint8_t payload[4] = {RxData[0], RxData[1], RxData[2], RxData[3]};
            SendFrame(CMD_EXTERNAL_LIGHT, payload);
            break;
        }

        default:
            break;
    }
}

/* ==================== SRS-UART-03 ====================
 * Reconstruit une trame UART octet par octet via une machine à états,
 * resynchronisée sur le SOF (0xA5). */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        switch (state)
        {
            case WAIT_FOR_SOF:
                if (rx_byte == SOF_PATTERN) state = WAIT_FOR_CMD;
                break;
            case WAIT_FOR_CMD:
                RxFrame.cmd = rx_byte;
                state = WAIT_FOR_DATA1;
                break;
            case WAIT_FOR_DATA1:
                RxFrame.payload[0] = rx_byte;
                state = WAIT_FOR_DATA2;
                break;
            case WAIT_FOR_DATA2:
                RxFrame.payload[1] = rx_byte;
                state = WAIT_FOR_DATA3;
                break;
            case WAIT_FOR_DATA3:
                RxFrame.payload[2] = rx_byte;
                state = WAIT_FOR_DATA4;
                break;
            case WAIT_FOR_DATA4:
                RxFrame.payload[3] = rx_byte;
                NewFrameReceivedCallback(&RxFrame);
                state = WAIT_FOR_SOF;
                break;
        }
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

/* Envoie une trame 6 octets (SOF+CMD+PAYLOAD) vers le dashboard PC. */
void SendFrame(uint8_t cmd, uint8_t *payload)
{
    uint8_t frame[6];
    frame[0] = SOF_PATTERN;
    frame[1] = cmd;
    frame[2] = payload[0];
    frame[3] = payload[1];
    frame[4] = payload[2];
    frame[5] = payload[3];
    HAL_UART_Transmit(&huart3, frame, 6, HAL_MAX_DELAY);
}

/* ==================== SRS-UART-04 ====================
 * Décode le CMD reçu du dashboard et déclenche l'action locale. */
void NewFrameReceivedCallback(FrameCom *rxFrame)
{
    uint8_t ack_payload[4] = {0, 0, 0, 0};
    switch (rxFrame->cmd)
    {
        case CMD_LED_ON:
            HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
            SendFrame(CMD_ACK, ack_payload);
            break;

        case CMD_LED_OFF:
            HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
            SendFrame(CMD_ACK, ack_payload);
            break;

        case CMD_READ_SENSOR:
        {
            uint8_t payload[4] = {0, 0, 0, 0};
            payload[3] = TxData[3];
            SendFrame(CMD_SENSOR_VALUE, payload);
            break;
        }

        /* ==================== SRS-CAN-04 ====================
         * Commande PC -> émission CAN directe (trigger ABS). */
        case CMD_TRIGGER_ABS:
        {
            CAN_TxHeaderTypeDef absHeader;
            absHeader.StdId = CANID_ABS;
            absHeader.IDE = CAN_ID_STD;
            absHeader.RTR = CAN_RTR_DATA;
            absHeader.DLC = 1;
            absHeader.TransmitGlobalTime = DISABLE;

            uint8_t absData[1] = {0x01};
            uint32_t absMailbox;
            if (HAL_CAN_AddTxMessage(&hcan1, &absHeader, absData, &absMailbox) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        }

        default:
            break;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
