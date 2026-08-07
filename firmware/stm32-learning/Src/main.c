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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint16_t bno055_address = 0;
static bool imu_log_enabled = false;
static ServoBus servo_bus;
static RobotController robot;
static AppConsole console;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void uart_print(const char *text);
static uint16_t BNO055_Detect(void);
static void I2C_Scan(void);
static HAL_StatusTypeDef BNO055_Write8(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef BNO055_Init(void);
static HAL_StatusTypeDef BNO055_ReadEuler(int16_t *heading,
                                         int16_t *roll,
                                         int16_t *pitch);
static bool BNO055_ReadAttitude(void *context,
                                int16_t *roll_tenths,
                                int16_t *pitch_tenths);
static void BNO055_PrintEuler(void);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  servo_bus_init(&servo_bus, &huart1, 25U);
  robot_init(&robot, &servo_bus);
  app_console_init(&console, &huart2, &robot, &imu_log_enabled);

  uart_print("\r\nPROGRAM START\r\n");
  HAL_Delay(700);
  I2C_Scan();

  bno055_address = BNO055_Detect();
  if (bno055_address == 0) {
      uart_print("BNO055 not found\r\n");
      uart_print("Trot/jump locked: use balance off only for explicit open-loop test\r\n");
  } else if (BNO055_Init() == HAL_OK) {
      uart_print("BNO055 NDOF initialization OK\r\n");
      robot_set_attitude_reader(&robot, BNO055_ReadAttitude, NULL);
      uart_print("IMU balance default ON: full, absolute level target\r\n");
  } else {
      uart_print("BNO055 initialization failed\r\n");
      uart_print("Trot/jump locked: use balance off only for explicit open-loop test\r\n");
  }
  app_console_print_help(&console);
  app_console_print_prompt(&console);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_imu_print = 0U;

    app_console_poll(&console);
    if (imu_log_enabled && bno055_address != 0 &&
        (uint32_t)(HAL_GetTick() - last_imu_print) >= 100U) {
        BNO055_PrintEuler();
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)

{

    if (GPIO_Pin == GPIO_PIN_13)

    {

        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{
    app_console_on_rx_complete(&console, huart);
}

static void uart_print(const char *text)
{
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)text,
        strlen(text),
        HAL_MAX_DELAY
    );
}

static uint16_t BNO055_Detect(void)
{
    const uint8_t addresses[] = {0x28, 0x29};
    uint8_t chip_id;
    char message[64];

    for (uint32_t i = 0; i < 2; i++) {
        uint16_t address = addresses[i] << 1;

        if (HAL_I2C_IsDeviceReady(
                &hi2c1,
                address,
                3,
                100) == HAL_OK) {

            if (HAL_I2C_Mem_Read(
                    &hi2c1,
                    address,
                    0x00,
                    I2C_MEMADD_SIZE_8BIT,
                    &chip_id,
                    1,
                    100) == HAL_OK) {

                snprintf(
                    message,
                    sizeof(message),
                    "Address: 0x%02X, CHIP_ID: 0x%02X\r\n",
                    addresses[i],
                    chip_id
                );

                uart_print(message);

                if (chip_id == 0xA0) {
                    return address;
                }
            }
        }
    }

    uart_print("BNO055 not found\r\n");
    return 0;
}

static void I2C_Scan(void)
{
    char msg[64];
    uint8_t found = 0;

    uart_print("\r\nI2C scan start\r\n");

    for (uint8_t addr = 1; addr < 127; addr++) {
        if (HAL_I2C_IsDeviceReady(
                &hi2c1,
                addr << 1,
                2,
                20) == HAL_OK) {

            snprintf(msg, sizeof(msg),
                     "Found: 0x%02X\r\n", addr);
            uart_print(msg);
            found++;
        }
    }

    if (found == 0) {
        uart_print("No I2C devices found\r\n");
    }

    uart_print("I2C scan finished\r\n");
}

#define BNO055_CHIP_ID_ADDR       0x00
#define BNO055_PAGE_ID_ADDR       0x07
#define BNO055_EULER_H_LSB_ADDR   0x1A
#define BNO055_OPR_MODE_ADDR      0x3D
#define BNO055_PWR_MODE_ADDR      0x3E
#define BNO055_SYS_TRIGGER_ADDR   0x3F

#define BNO055_MODE_CONFIG        0x00
#define BNO055_MODE_NDOF          0x0C
#define BNO055_POWER_NORMAL       0x00

static HAL_StatusTypeDef BNO055_Write8(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             bno055_address,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1,
                             100);
}

static HAL_StatusTypeDef BNO055_Init(void)
{
    uint8_t chip_id = 0;

    if (bno055_address == 0) {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Read(&hi2c1,
                         bno055_address,
                         BNO055_CHIP_ID_ADDR,
                         I2C_MEMADD_SIZE_8BIT,
                         &chip_id,
                         1,
                         100) != HAL_OK || chip_id != 0xA0) {
        return HAL_ERROR;
    }

    if (BNO055_Write8(BNO055_OPR_MODE_ADDR,
                      BNO055_MODE_CONFIG) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(25);

    if (BNO055_Write8(BNO055_PAGE_ID_ADDR, 0x00) != HAL_OK ||
        BNO055_Write8(BNO055_PWR_MODE_ADDR,
                      BNO055_POWER_NORMAL) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(10);

    if (BNO055_Write8(BNO055_SYS_TRIGGER_ADDR, 0x00) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(10);

    if (BNO055_Write8(BNO055_OPR_MODE_ADDR,
                      BNO055_MODE_NDOF) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(30);

    return HAL_OK;
}

static HAL_StatusTypeDef BNO055_ReadEuler(int16_t *heading,
                                         int16_t *roll,
                                         int16_t *pitch)
{
    uint8_t data[6];

    if (HAL_I2C_Mem_Read(&hi2c1,
                         bno055_address,
                         BNO055_EULER_H_LSB_ADDR,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         sizeof(data),
                         100) != HAL_OK) {
        return HAL_ERROR;
    }

    *heading = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    *roll = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    *pitch = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    return HAL_OK;
}

static bool BNO055_ReadAttitude(void *context,
                                int16_t *roll_tenths,
                                int16_t *pitch_tenths)
{
    int16_t heading_raw = 0;
    int16_t roll_raw = 0;
    int16_t pitch_raw = 0;
    (void)context;

    if (roll_tenths == NULL || pitch_tenths == NULL ||
        BNO055_ReadEuler(&heading_raw, &roll_raw, &pitch_raw) != HAL_OK) {
        return false;
    }

    /* BNO055 Euler scale: 16 LSB per degree. Yaw is intentionally unused. */
    *roll_tenths = (int16_t)(((int32_t)roll_raw * 10) / 16);
    *pitch_tenths = (int16_t)(((int32_t)pitch_raw * 10) / 16);
    return true;
}

static void BNO055_PrintEuler(void)
{
    int16_t heading_raw;
    int16_t roll_raw;
    int16_t pitch_raw;
    int32_t heading10;
    int32_t roll10;
    int32_t pitch10;
    char message[100];

    if (BNO055_ReadEuler(&heading_raw,
                        &roll_raw,
                        &pitch_raw) != HAL_OK) {
        uart_print("Euler read error\r\n");
        return;
    }

    /* BNO055 Euler scale: 16 LSB per degree. */
    heading10 = ((int32_t)heading_raw * 10) / 16;
    roll10 = ((int32_t)roll_raw * 10) / 16;
    pitch10 = ((int32_t)pitch_raw * 10) / 16;

    snprintf(message,
             sizeof(message),
             "Yaw=%ld.%01ld, Roll=%s%ld.%01ld, Pitch=%s%ld.%01ld deg\r\n",
             (long)(heading10 / 10),
             (long)labs(heading10 % 10),
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
