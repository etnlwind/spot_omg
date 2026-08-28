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
#include "app_console.h"
#include "bno055.h"
#include "bno086.h"
#include "robot.h"
#include "servo_bus.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/*
 * Both IMUs are compiled in and the attached one is picked at boot.  The
 * BNO055 lives on I2C1 and the BNO086 on SPI1, so they cannot collide, and
 * swapping the sensor no longer means reflashing a different build.
 */
static Bno055 imu055;
static Bno086 imu086;
static bool imu_log_enabled = false;
static ServoBus servo_bus;
static RobotController robot;
static AppConsole console;
static AppConsole wifi_console;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void uart_print(const char *text);
static void print_bno086_bringup(const Bno086 *imu, Bno086Result result);
static void IMU_PrintEuler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const char *gpio_level(bool active_low_asserted)
{
  return active_low_asserted ? "LOW" : "HIGH";
}

static void print_bno086_bringup(const Bno086 *imu, Bno086Result result)
{
  char message[180];

  (void)snprintf(message,
                 sizeof(message),
                 "BNO086 RESET: INT before=%s during=%s after=%s; first LOW=%s",
                 gpio_level(imu->int_before_reset),
                 gpio_level(imu->int_during_reset),
                 gpio_level(imu->int_after_boot_wait),
                 imu->first_interrupt_seen ? "yes" : "no");
  uart_print(message);
  if (imu->first_interrupt_seen)
  {
    (void)snprintf(message,
                   sizeof(message),
                   " at %lums\r\n",
                   (unsigned long)imu->first_interrupt_delay_ms);
    uart_print(message);
    (void)snprintf(message,
                   sizeof(message),
                   "BNO055 calibration: device=%s level=%s zero=(%d,%d) tenths\r\n",
                   imu055.device_profile_restored ? "restored" : "none",
                   imu055.level_valid ? "restored" : "none",
                   (int)imu055.level_roll_tenths,
                   (int)imu055.level_pitch_tenths);
    uart_print(message);
  }
  else
  {
    uart_print("\r\n");
  }

  (void)snprintf(message,
                 sizeof(message),
                 "BNO086 SHTP: first=%s header=%02X %02X %02X %02X "
                 "len=%u channel=%u seq=%u packets=%lu invalid=%lu spierr=%lu\r\n",
                 imu->first_packet_seen ? "valid" : "none",
                 (unsigned int)imu->first_header[0],
                 (unsigned int)imu->first_header[1],
                 (unsigned int)imu->first_header[2],
                 (unsigned int)imu->first_header[3],
                 (unsigned int)imu->first_packet_length,
                 (unsigned int)imu->first_packet_channel,
                 (unsigned int)imu->first_packet_sequence,
                 (unsigned long)imu->packets_received,
                 (unsigned long)imu->invalid_headers,
                 (unsigned long)imu->spi_errors);
  uart_print(message);

  (void)snprintf(message,
                 sizeof(message),
                 "BNO086 Product ID: status=%d entries=%u\r\n",
                 imu->product_id_status,
                 (unsigned int)imu->product_ids.numEntries);
  uart_print(message);
  for (uint8_t index = 0U; index < imu->product_ids.numEntries; ++index)
  {
    const sh2_ProductId_t *id = &imu->product_ids.entry[index];
    (void)snprintf(message,
                   sizeof(message),
                   "  product[%u]: part=%lu version=%u.%u.%u build=%lu reset=%u\r\n",
                   (unsigned int)index,
                   (unsigned long)id->swPartNumber,
                   (unsigned int)id->swVersionMajor,
                   (unsigned int)id->swVersionMinor,
                   (unsigned int)id->swVersionPatch,
                   (unsigned long)id->swBuildNumber,
                   (unsigned int)id->resetCause);
    uart_print(message);
  }

  if (result == BNO086_OK && imu->has_attitude)
  {
    (void)snprintf(message,
                   sizeof(message),
                   "BNO086 Rotation Vector: reports=%lu "
                   "q_x10000=(%ld,%ld,%ld,%ld) angles_tenths=(%d,%d,%d)\r\n",
                   (unsigned long)imu->report_count,
                   (long)(imu->quat_i * 10000.0f),
                   (long)(imu->quat_j * 10000.0f),
                   (long)(imu->quat_k * 10000.0f),
                   (long)(imu->quat_real * 10000.0f),
                   (int)imu->roll_tenths,
                   (int)imu->pitch_tenths,
                   (int)imu->yaw_tenths);
    uart_print(message);
  }

  (void)snprintf(message,
                 sizeof(message),
                 "BNO086 bring-up result: %s\r\n",
                 bno086_result_string(result));
  uart_print(message);
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  servo_bus_init(&servo_bus, &huart1, 25U);
  robot_init(&robot, &servo_bus);
  app_console_init(&console, &huart2, &robot, &imu055, &imu086, &imu_log_enabled);
  app_console_init(&wifi_console, &huart3, &robot, &imu055, &imu086, &imu_log_enabled);

  uart_print("\r\nPROGRAM START\r\n");
  HAL_Delay(700);

  /*
   * Try the BNO055 first. Its probe is a couple of short I2C transfers, while
   * a BNO086 that is absent or silent costs seconds of SH-2 timeouts, so this
   * order keeps boot quick in the common case.
   */
  uart_print("Waveshare servo bus ready: USART1 at 1000000 baud\r\n");
  imu055.i2c = &hi2c1;
  if (bno055_init(&imu055, &hi2c1))
  {
    char message[128];
    (void)snprintf(message,
                   sizeof(message),
                   "BNO055 IMUPLUS OK at 0x%02X\r\n",
                   (unsigned int)(imu055.address >> 1));
    uart_print(message);
    robot_set_attitude_reader(&robot, bno055_read_attitude, &imu055);
    uart_print("IMU balance default ON: full, absolute level target\r\n");
  }
  else
  {
    /*
     * 5 ms subscription: the sensor fuses faster than the balance loop
     * consumes, so a step never acts on a sample older than one period.
     */
    uart_print("BNO086 bring-up: SPI1 mode 3, 1MHz; CS=PB6 RST=PB2 INT=PA8(active-low)\r\n");
    const Bno086Result imu_result = bno086_init(&imu086, &hspi1, 5000U);
    print_bno086_bringup(&imu086, imu_result);
    if (imu_result == BNO086_OK)
    {
      uart_print("BNO086 game rotation vector OK at 200Hz\r\n");
      robot_set_attitude_reader(&robot, bno086_read_attitude, &imu086);
      uart_print("IMU balance default ON: full, absolute level target\r\n");
    }
    else
    {
      char message[80];
      (void)snprintf(message,
                     sizeof(message),
                     "No IMU: BNO055 absent, BNO086 %s (prodIds=%d)\r\n",
                     bno086_result_string(imu_result),
                     imu086.product_id_status);
      uart_print(message);
      uart_print("Trot/jump locked: use balance off only for explicit open-loop test\r\n");
    }
  }

  app_console_print_help(&console);
  app_console_print_prompt(&console);

  app_console_print_help(&wifi_console);
  app_console_print_prompt(&wifi_console);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_imu_print = 0U;
    //      static uint32_t uart3_test_tick = 0U;

    // /* USART3 -> ESP32 단방향 테스트 */
    // if ((uint32_t)(HAL_GetTick() - uart3_test_tick) >= 1000U)
    // {
    //     uart3_test_tick = HAL_GetTick();

    //     const char msg[] = "STM32 USART3 TX OK\r\n";

    //     HAL_UART_Transmit(
    //         &huart3,
    //         (uint8_t *)msg,
    //         sizeof(msg) - 1U,
    //         100U
    //     );
    // }

    app_console_poll(&console);
    app_console_poll(&wifi_console);

    bno086_service(&imu086);

    if (imu_log_enabled && (imu055.present || imu086.present) &&
        (uint32_t)(HAL_GetTick() - last_imu_print) >= 100U)
    {
      IMU_PrintEuler();
      last_imu_print = HAL_GetTick();
    }

    HAL_Delay(1);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /*
   * The BNO086 samples on the trailing edge of an idle-high clock, so mode 3.
   * PCLK2 is 16 MHz and /8 gives 2 MHz, under the part's 3 MHz ceiling.
   */
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 1000000;
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

  /*
   * STS3215 uses a 1 Mbps bus. Keep this runtime guard in a USER CODE
   * section so a CubeMX regeneration cannot silently leave USART1 at its
   * 115200 default.
   */
  if (huart1.Init.BaudRate != 1000000U)
  {
    huart1.Init.BaudRate = 1000000U;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      Error_Handler();
    }
  }

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IMU_RST_Pin|IMU_WAKE_Pin|IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_RST_Pin IMU_WAKE_Pin */
  GPIO_InitStruct.Pin = IMU_RST_Pin|IMU_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 * The B1 user button on PC13 used to blink LD2.  LD2 sits on PA5, which is now
 * SPI1_SCK for the BNO086, so the callback is gone rather than left blinking
 * an LED that would corrupt every SPI frame.
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{
  app_console_on_rx_complete(&console, huart);
  app_console_on_rx_complete(&wifi_console, huart);
}

static void uart_print(const char *text)
{
  HAL_UART_Transmit(
      &huart2,
      (uint8_t *)text,
      strlen(text),
      HAL_MAX_DELAY);
}

static void IMU_PrintEuler(void)
{
  char message[100];
  int32_t yaw10 = 0;
  int32_t roll10 = 0;
  int32_t pitch10 = 0;

  if (imu055.present)
  {
    int16_t yaw = 0;
    int16_t roll = 0;
    int16_t pitch = 0;
    if (!bno055_read_euler(&imu055, &yaw, &roll, &pitch))
    {
      uart_print("Euler read error\r\n");
      return;
    }
    yaw10 = yaw;
    roll10 = roll;
    pitch10 = pitch;
  }
  else if (imu086.present && imu086.has_attitude)
  {
    yaw10 = imu086.yaw_tenths;
    roll10 = imu086.roll_tenths;
    pitch10 = imu086.pitch_tenths;
  }
  else
  {
    uart_print("IMU unavailable\r\n");
    return;
  }

  /*
   * Same line shape the BNO055 has always printed, so the host-side parsers
   * and the bench notes in HARDWARE_TEST_LOG.md keep working.  Note the yaw
   * differs by sensor: absolute heading on the BNO055, relative to power-on
   * on the BNO086's game rotation vector.
   */
  snprintf(message,
           sizeof(message),
           "Yaw=%s%ld.%01ld, Roll=%s%ld.%01ld, Pitch=%s%ld.%01ld deg\r\n",
           yaw10 < 0 ? "-" : "",
           labs(yaw10) / 10,
           labs(yaw10) % 10,
           roll10 < 0 ? "-" : "",
           labs(roll10) / 10,
           labs(roll10) % 10,
           pitch10 < 0 ? "-" : "",
           labs(pitch10) / 10,
           labs(pitch10) % 10);

  uart_print(message);
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
