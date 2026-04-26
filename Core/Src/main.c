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
#include "a7672e.h"

#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ── printf → USART1 (debug) ─────────────────────────────────────────── */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}

/* ── MQTT downlink callback ──────────────────────────────────────────── */
static void on_mqtt_message(const char *topic, const char *payload,
                             uint16_t payload_len)
{
    printf("[MQTT RX] topic: %s\r\n", topic);
    printf("[MQTT RX] payload (%u B): %.*s\r\n",
           (unsigned)payload_len, (int)payload_len, payload);

    /* ── Command dispatch on device/commands ─────────────────────────── *
     * Expected JSON:  {"cmd":"output0","val":1}                          *
     * Extend this section to handle your downlink commands.              */
    if (strstr(payload, "\"output0\"")) {
        uint8_t state = strstr(payload, "\"val\":1") ? 1u : 0u;
        HAL_GPIO_WritePin(output0_GPIO_Port, output0_Pin,
                          state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        printf("[CMD] output0 -> %s\r\n", state ? "ON" : "OFF");
    }
    else if (strstr(payload, "\"Output1\"")) {
        uint8_t state = strstr(payload, "\"val\":1") ? 1u : 0u;
        HAL_GPIO_WritePin(Output1_GPIO_Port, Output1_Pin,
                          state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        printf("[CMD] Output1 -> %s\r\n", state ? "ON" : "OFF");
    }
    else if (strstr(payload, "\"Output2\"")) {
        uint8_t state = strstr(payload, "\"val\":1") ? 1u : 0u;
        HAL_GPIO_WritePin(Output2_GPIO_Port, Output2_Pin,
                          state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        printf("[CMD] Output2 -> %s\r\n", state ? "ON" : "OFF");
    }
}

/* ── MQTT connection-lost callback ──────────────────────────────────── */
static volatile uint8_t g_mqtt_reconnect = 0;

static void on_mqtt_lost(void)
{
    printf("[MQTT] connection lost — will reconnect\r\n");
    g_mqtt_reconnect = 1;
}

/* ── Custom hex payloads cycled to a7672/data every 60 s ────────────── */
static const char * const g_payloads[] = {
    "68250100103254769881239025012b140000002b50000000000000000004ddccbbaa502800005523270426200000b916",
    "68250100103254769881239025022b280000002b3c000000000000000004ddccbbaa2029000025002704262000003816",
    "68250100103254769881239025032b3c0000002b28000000000000000004ddccbbaa1529000055002704262000005e16",
    "68250100103254769881239025042b500000002b14000000000000000004ddccbbaa902800002501270426200000aa16",
    "68250100103254769881239025052b500000002b0000000000000000000744332211252900005501270426200000fc16",
    "68250100103254769881239025062b640000002b1e00000000000000000212903cfa3029000025022704262000003416",
    "68250100103254769881239025072b780000002b0a00000000000000000212903cfa1029000055022704262000004516",
    "68250100103254769881239025082b8c0000002b28000000000000000004ddccbbaa752800002503270426200000e516",
    "68250100103254769881239025092ba00000002b14000000000000000004ddccbbaa6028000055032704262000000116",
    "682501001032547698812390250a2ba00000002b00000000000000000007443322115028000025042704262000004e16",
    "682501001032547698812390250b2baa0000002b0000000000000000000212903cfa802800005504270426200000e216",
    "682501001032547698812390250c2bbe0000002b32000000000000000003887766550029000025052704262000005e16",
    "682501001032547698812390250d2bd20000002b1e000000000000000003887766550529000055052704262000009416",
    "682501001032547698812390250e2be60000002b0a00000000000000000388776655952800002506270426200000f516",
    "682501001032547698812390250f2bfa0000002b00000000000000000004ddccbbaa7028000055062704262000006016",
    "68250100103254769881239025102b0e0100002b1400000000000000000212903cfa6528000025072704262000001816",
    "68250100103254769881239025112b220100002b0000000000000000000212903cfa6028000055072704262000004416",
    "68250100103254769881239025122b360100002b2800000000000000000744332211102900002508270426200000da16",
    "68250100103254769881239025132b4a0100002b14000000000000000007443322112029000055082704262000001b16",
    "68250100103254769881239025142b5e0100002b0000000000000000000744332211152900002509270426200000e216",
};
#define PAYLOAD_COUNT  (sizeof(g_payloads) / sizeof(g_payloads[0]))

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* ── Broker: remote Mosquitto (plain TCP 1883) ────────────────────── */
  static const A7672E_MqttConfig_t mqtt_cfg = {
      .broker    = "157.173.107.18",
      .port      = 1883,
      .client_id = "A7672Client",
      .username  = "admin",
      .password  = "mugiendii",
      .use_ssl   = 0,
  };

  /* Network configuration — from your ESP32 implementation */
  static const char *A7672E_PIN = "3759";        /* Your Safaricom SIM PIN */
  static const char *A7672E_APN = "safaricom";   /* Safaricom APN */

  /* Topics (mirrors setup.py) */
  #define TOPIC_STATUS        "a7672/status"
  #define TOPIC_DATA          "a7672/data"
  #define TOPIC_COMMANDS      "a7672/#"

  printf("== A7672E LTE HAL starting ==\r\n");

  /* ── Initialise driver, register callbacks ──────────────────────────── */
  A7672E_Init();
  A7672E_MqttSetMsgCallback(on_mqtt_message);
  A7672E_MqttSetLostCallback(on_mqtt_lost);

  A7672E_PowerOn();
  printf("Waiting for modem...\r\n");

  if (A7672E_WaitReady(15000) != A7672E_OK) {
      printf("ERROR: modem not responding\r\n");
      Error_Handler();
  }
  printf("Modem OK\r\n");


  /* ── Network registration ──────────────────────────── */
  printf("Registering on network...\r\n");
  A7672E_Status_t net_st = A7672E_InitNetwork(A7672E_PIN, A7672E_APN);
  if (net_st != A7672E_OK) {
      const char *reason = (net_st == A7672E_NO_SIM) ? "no SIM / SIM removed" :
                           (net_st == A7672E_NO_NET) ? "registration timeout"  :
                                                       "PDP activation failed";
      printf("ERROR: network init failed — %s\r\n", reason);
      Error_Handler();
  }
  printf("Network ready\r\n");

  /* ── MQTT connect ───────────────────────────────────────────────────── */
  printf("Connecting to local Mosquitto...\r\n");
  if (A7672E_MqttConnect(&mqtt_cfg) != A7672E_OK) {
      printf("ERROR: MQTT connect failed\r\n");
      Error_Handler();
  }
  printf("MQTT connected\r\n");

  /* Announce online */
  A7672E_MqttPublish(TOPIC_STATUS, "Device connected", 1, 0);

  /* ── Subscribe to all a7672 topics ────────────────────────────────── */
  if (A7672E_MqttSubscribe(TOPIC_COMMANDS, 1) != A7672E_OK) {
      printf("ERROR: subscribe failed\r\n");
      Error_Handler();
  }
  printf("Subscribed to %s\r\n", TOPIC_COMMANDS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_heartbeat = HAL_GetTick();
  uint32_t last_payload   = HAL_GetTick();   /* first payload fires after 60 s */
  uint32_t payload_index  = 0;

  /* Send first payload immediately (mirrors setup.py behaviour) */
  printf("[TX] Sending payload 1/%u to %s\r\n", (unsigned)PAYLOAD_COUNT, TOPIC_DATA);
  if (A7672E_MqttPublish(TOPIC_DATA, g_payloads[0], 1, 0) == A7672E_OK)
      printf("[TX] OK\r\n");
  else
      printf("[TX] FAILED\r\n");
  payload_index = 1;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Drive URC state machine every loop iteration */
    A7672E_Process();

    /* Reconnect if broker dropped the connection */
    if (g_mqtt_reconnect) {
        g_mqtt_reconnect = 0;
        printf("Reconnecting...\r\n");
        if (A7672E_MqttConnect(&mqtt_cfg) == A7672E_OK) {
            A7672E_MqttPublish(TOPIC_STATUS, "Device connected", 1, 0);
            A7672E_MqttSubscribe(TOPIC_COMMANDS, 1);
        }
    }

    /* ── Heartbeat to a7672/status every 30 s ────────────────────── */
    if ((HAL_GetTick() - last_heartbeat) >= 30000u) {
        last_heartbeat = HAL_GetTick();
        char hbuf[48];
        snprintf(hbuf, sizeof(hbuf), "{\"uptime_ms\":%lu}",
                 (unsigned long)HAL_GetTick());
        if (A7672E_MqttPublish(TOPIC_STATUS, hbuf, 0, 0) == A7672E_OK)
            printf("[HB] sent: %s\r\n", hbuf);
        else
            printf("[HB] FAILED\r\n");
    }

    /* ── Hex payload to a7672/data every 60 s (cycle through all 20) ── */
    if ((HAL_GetTick() - last_payload) >= 60000u) {
        last_payload = HAL_GetTick();
        if (payload_index < PAYLOAD_COUNT) {
            printf("[TX] Sending payload %lu/%u...\r\n",
                   (unsigned long)(payload_index + 1), (unsigned)PAYLOAD_COUNT);
            if (A7672E_MqttPublish(TOPIC_DATA, g_payloads[payload_index], 1, 0) == A7672E_OK)
                printf("[TX] OK\r\n");
            else
                printf("[TX] FAILED\r\n");
            payload_index++;
        } else {
            printf("[TX] All %u payloads sent. Monitoring...\r\n",
                   (unsigned)PAYLOAD_COUNT);
        }
    }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, output0_Pin|Output1_Pin|Output2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_KEY_GPIO_Port, PWR_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : output0_Pin Output1_Pin Output2_Pin */
  GPIO_InitStruct.Pin = output0_Pin|Output1_Pin|Output2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_KEY_Pin */
  GPIO_InitStruct.Pin = PWR_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN0_Pin */
  GPIO_InitStruct.Pin = IN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  UART RX complete — forward modem UART bytes into the driver ring buffer.
 *         USART1 is debug-only (TX), no RX callback needed for it.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
        A7672E_UART_RxCpltCallback();
}

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
