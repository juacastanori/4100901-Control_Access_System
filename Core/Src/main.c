/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with updated command, timer, LED ring,
  *                   OLED bitmap display logic, and sleep mode functionality.
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
#include "keypad.h"
#include "ring_buffer.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <string.h>
#include <stdio.h>
#include "ring.h"
#include "locked.h"
#include "unlocked.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FW_VERSION "0.1.0\r\n"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t rx_byte;
uint8_t start = 0;

/* --- Globals for door state --- */
uint8_t door_permanent = 0;  // 0 = temporary/closed; 1 = permanently open

/* --- Global for temporary open timing (5 sec) --- */
uint32_t temp_open_start = 0;

/* --- For button debouncing and tracking --- */
uint32_t key_pressed_tick = 0;
uint16_t column_pressed = 0;
uint8_t byte_received_uart2;
uint8_t byte_received_uart3;
uint8_t button_press_count = 0;
uint32_t last_button_press_time = 0;
uint32_t debounce_tick = 0;
uint32_t button_debounce_tick = 0;

/* --- New globals for sleep/inactivity and mistakes --- */
uint32_t last_activity_tick = 0;    // updated on every activity (button or UART)
uint8_t mistake_count = 0;          // counts invalid (unrecognized) commands

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  uint32_t current_tick = HAL_GetTick();
  last_activity_tick = current_tick;  // any external interrupt resets inactivity timer
  if (GPIO_Pin == B1_Pin) {
    if ((current_tick - button_debounce_tick) < 200) {
      return;
    }
    button_debounce_tick = current_tick;
    button_press_count++;
    last_button_press_time = current_tick;
  }
  else if (GPIO_Pin == B2_Pin) {
    // No immediate action; B2 is polled in main loop.
  }
  else {
    if ((debounce_tick + 200) > current_tick) {
      return;
    }
    debounce_tick = current_tick;
    key_pressed_tick = current_tick;
    column_pressed = GPIO_Pin;
  }
}


#define COMMAND_LENGTH 5
const char CMD_START[]     = "#*#*#";
const char CMD_TEMP_OPEN[] = "#*A*#";   // Temporary open command (5 sec)
const char CMD_CLOSE[]     = "#*C*#";   // Close command
const char CMD_STATUS[]    = "#*1*#";   // Status command
const char CMD_RESET[]     = "#*0*#";   // Reset command

ring_buffer_t rx_buffer;
uint8_t rx_buffer_mem[64];
char current_cmd[COMMAND_LENGTH];
uint8_t cmd_index = 0;

/* --- UART Receive Callback --- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  uint32_t current_tick = HAL_GetTick();
  last_activity_tick = current_tick;  // activity detected from UART
  if (huart == &huart2) {
    ring_buffer_write(&rx_buffer, byte_received_uart2);
    HAL_UART_Transmit(&huart3, &byte_received_uart2, 1, 10);
    HAL_UART_Receive_IT(&huart2, &byte_received_uart2, 1);
  } else if (huart == &huart3) {
    ring_buffer_write(&rx_buffer, byte_received_uart3);
    HAL_UART_Transmit(&huart2, &byte_received_uart3, 1, 10);
    HAL_UART_Receive_IT(&huart3, &byte_received_uart3, 1);
  }
}


void sleep_mode_inactivity(void) {
  uart_send_string("\r\nNo activity for 30 sec. Entering sleep mode.\r\n");
  HAL_SuspendTick();
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  HAL_ResumeTick();
  uart_send_string("\r\nAwake from inactivity sleep.\r\n");
  last_activity_tick = HAL_GetTick();
}


void sleep_mode_mistake(void) {
  uart_send_string("\r\nToo many invalid commands. Sleeping for 10 sec.\r\n");
  HAL_SuspendTick();
  uint32_t sleepStart = HAL_GetTick();
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  // Ensure at least 10 sec pass before resuming
  while(HAL_GetTick() - sleepStart < 10000) { }
  HAL_ResumeTick();
  uart_send_string("\r\nAwake from mistake sleep.\r\n");
  mistake_count = 0;
  last_activity_tick = HAL_GetTick();
}


void process_commands(void) {
  uint8_t byte;
  while (ring_buffer_read(&rx_buffer, &byte)) {
    memmove(current_cmd, current_cmd + 1, COMMAND_LENGTH - 1);
    current_cmd[COMMAND_LENGTH - 1] = (char)byte;

    if (memcmp(current_cmd, CMD_START, COMMAND_LENGTH) == 0) {
      start = 1;
      uart_send_string("\r\nCommand mode activated. Send commands.\r\n");
      memset(current_cmd, 0, COMMAND_LENGTH);
      mistake_count = 0;  // reset mistakes on valid start
    }

    if (start) {
      if (memcmp(current_cmd, CMD_TEMP_OPEN, COMMAND_LENGTH) == 0) {
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
        door_permanent = 0;
        temp_open_start = HAL_GetTick();
        uart_send_string("\r\nDoor opened temporarily (5 sec).\r\n");
        memset(current_cmd, 0, COMMAND_LENGTH);
        mistake_count = 0;
      }
      else if (memcmp(current_cmd, CMD_CLOSE, COMMAND_LENGTH) == 0) {
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
        door_permanent = 0;
        temp_open_start = 0;
        uart_send_string("\r\nDoor closed.\r\n");
        memset(current_cmd, 0, COMMAND_LENGTH);
        mistake_count = 0;
      }
      else if (memcmp(current_cmd, CMD_STATUS, COMMAND_LENGTH) == 0) {
        uint8_t state = HAL_GPIO_ReadPin(LD4_GPIO_Port, LD4_Pin);
        uart_send_string(state ? "\r\nStatus: OPEN\r\n" : "\r\nStatus: CLOSED\r\n");
        memset(current_cmd, 0, COMMAND_LENGTH);
        mistake_count = 0;
      }
      else if (memcmp(current_cmd, CMD_RESET, COMMAND_LENGTH) == 0) {
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
        door_permanent = 0;
        temp_open_start = 0;
        uart_send_string("\r\nSystem reset.\r\n");
        memset(current_cmd, 0, COMMAND_LENGTH);
        mistake_count = 0;
      }

      else {

        if (current_cmd[0] != 0) {
          mistake_count++;
          char msg[50];
          sprintf(msg, "\r\nInvalid command. %d tries remaining until sleep.\r\n", 5 - mistake_count);
          uart_send_string(msg);
          memset(current_cmd, 0, COMMAND_LENGTH);
          if (mistake_count >= 5) {
            sleep_mode_mistake();
          }
        }
      }
    }
  }
}


void uart_send_string(const char *str) {
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
  HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 100);
}


void heartbeat(void)
{
  static uint32_t last_tick = 0;
  if ((last_tick + 500) < HAL_GetTick()) {
    last_tick = HAL_GetTick();
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  keypad_init();
  ring_buffer_init(&rx_buffer, rx_buffer_mem, sizeof(rx_buffer_mem));
  memset(current_cmd, 0, COMMAND_LENGTH);
  last_activity_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart2, &byte_received_uart2, 1);
  HAL_UART_Receive_IT(&huart3, &byte_received_uart3, 1);
  HAL_UART_Transmit(&huart2, (uint8_t *)FW_VERSION, strlen(FW_VERSION), 10);
  HAL_UART_Transmit(&huart3, (uint8_t *)FW_VERSION, strlen(FW_VERSION), 10);

  ssd1306_SetCursor(20, 20);
  ssd1306_WriteString((char *)FW_VERSION, Font_7x10, White);
  ssd1306_UpdateScreen();

  static uint32_t last_blink_tick = 0;
  static uint8_t prev_b2_state = 0;

  while (1)
  {
    heartbeat();

    if (HAL_GetTick() - last_activity_tick >= 30000) {
      sleep_mode_inactivity();
    }

    if (column_pressed != 0 && (key_pressed_tick + 5) < HAL_GetTick()) {
      uint8_t key = keypad_scan(column_pressed);
      ring_buffer_write(&rx_buffer, key);
      HAL_UART_Transmit(&huart2, &key, 1, 100);
      HAL_UART_Transmit(&huart3, &key, 1, 100);
      column_pressed = 0;
    }

    process_commands();

    /* --- Updated B1 button handling ---
         Single press: if door is not permanently open, open door temporarily (5 sec);
                       if door is permanently open, then close it.
         Double press: open door permanently.
    */
    if (button_press_count > 0 && (HAL_GetTick() - last_button_press_time) >= 500) {
      if (button_press_count == 1) {
        if (door_permanent) {
          HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
          door_permanent = 0;
          uart_send_string("\r\nButton: Door closed.\r\n");
        } else {
          HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
          temp_open_start = HAL_GetTick();
          uart_send_string("\r\nButton: Door opened temporarily (5 sec).\r\n");
        }
      } else if (button_press_count >= 2) {
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
        door_permanent = 1;
        temp_open_start = 0;
        uart_send_string("\r\nButton: Door opened permanently.\r\n");
      }
      button_press_count = 0;
      last_button_press_time = 0;
    }

    /* --- Auto-close temporary door after 5 seconds ---
         Only if door is not set to permanent.
    */
    if (!door_permanent && temp_open_start != 0 &&
        (HAL_GetTick() - temp_open_start >= 5000)) {
      HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
      uart_send_string("\r\nDoor auto-closed after temporary open.\r\n");
      temp_open_start = 0;
    }

    /* --- Ring Button (B2) and OLED display handling ---
         When B2 is pressed (GPIO high):
             - The ring LED (LD5) is forced OFF.
             - The OLED displays the ring bitmap.
             - On a rising edge, a UART message is sent.
         When B2 is not pressed:
             - The ring LED toggles every 250 ms.
             - The OLED displays the door state image:
                 > If the door (LD4) is ON, display the "unlocked" bitmap.
                 > Otherwise, display the "locked" bitmap.
    */
   uint8_t current_b2_state = HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin);
  if (current_b2_state == GPIO_PIN_RESET) {  // Button pressed (active low)
      if (!prev_b2_state) {
          uart_send_string("\r\nRing pressed.\r\n");
      }
      // Turn the LED ON when the button is pressed.
      HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
      ssd1306_Fill(Black);
      ssd1306_DrawBitmap(0, 0, ring, 128, 64, White);
      ssd1306_UpdateScreen();
      prev_b2_state = 1;
  } else {
      // Turn the LED OFF when the button is not pressed.
      HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
      ssd1306_Fill(Black);
      if (HAL_GPIO_ReadPin(LD4_GPIO_Port, LD4_Pin) == GPIO_PIN_SET) {
        ssd1306_DrawBitmap(0, 0, unlocked, 128, 64, White);
      } else {
        ssd1306_DrawBitmap(0, 0, locked, 128, 64, White);
      }
      ssd1306_UpdateScreen();
      prev_b2_state = 0;
  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD4_Pin|LD2_Pin|LD5_Pin|ROW_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ROW_2_Pin|ROW_4_Pin|ROW_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B2_Pin */
  GPIO_InitStruct.Pin = B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD2_Pin LD5_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD2_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : COLUMN_1_Pin */
  GPIO_InitStruct.Pin = COLUMN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COLUMN_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : COLUMN_4_Pin */
  GPIO_InitStruct.Pin = COLUMN_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COLUMN_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COLUMN_2_Pin COLUMN_3_Pin */
  GPIO_InitStruct.Pin = COLUMN_2_Pin|COLUMN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW_1_Pin */
  GPIO_InitStruct.Pin = ROW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ROW_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW_2_Pin ROW_4_Pin ROW_3_Pin */
  GPIO_InitStruct.Pin = ROW_2_Pin|ROW_4_Pin|ROW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
