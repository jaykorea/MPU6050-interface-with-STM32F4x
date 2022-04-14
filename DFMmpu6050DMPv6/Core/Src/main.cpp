/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef DEBUG
#include <string.h>
#include <stdio.h>
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR            (0x68 << 1)
#define TMP_REG                 0x41
#define MPU6050_ACCEL_XOUT_H		0x3B
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int16_t acx, acy, acz, tmp, gyx, gyy, gyz;
static int16_t offset_gyx = 0, offset_gyy = 0, offset_gyz = 0;
static float degrees_pitch_acc, degrees_roll_acc;
static float acc_vector;
static float degrees_pitch = 0, degrees_roll = 0;
static uint32_t _millis = 0;

static uint8_t serialBuffer[80];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void calibirate_MPU6050(int num_iterations);
void read_MPU6050(void);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init();
  calibirate_MPU6050(3000);

  _millis = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  read_MPU6050();
	     gyx -= offset_gyx;
	     gyy -= offset_gyy;
	     gyz -= offset_gyz;

	     degrees_pitch += gyy * 0.0000610687;
	     degrees_roll  += gyx * 0.0000610687;

	     degrees_pitch += degrees_roll * sin(gyz * 0.000001066);
	     degrees_roll  -= degrees_pitch * sin(gyz * 0.000001066);

	     acc_vector = sqrt((acx * acx) + (acy * acy) + (acz * acz));
	     degrees_pitch_acc = asin((float) acy/acc_vector) * 57.2957795;
	     degrees_roll_acc  = asin((float) acx/acc_vector) * -57.2957795;

	     degrees_pitch = degrees_pitch * 0.97 + degrees_pitch_acc * 0.03;
	     degrees_roll  = degrees_roll * 0.97 + degrees_roll_acc * 0.03;

	     float temp = (tmp / 340.00 + 36.53) * 100;
	     float formatted_pitch = degrees_pitch * 100;
	     float formatted_roll = degrees_roll * 100;

	     sprintf(
	       (char *)serialBuffer, "Pitch: %d.%02u, Roll: %d.%02u, Temp: %d.%02u\r\n",
	       (int)formatted_pitch / 100,
	       (unsigned int)formatted_pitch % 100,
	       (int)formatted_roll / 100,
	       (unsigned int)formatted_roll % 100,
	       (int)temp / 100,
	       (unsigned int)temp % 100
	     );
	     printf("%f\r\n", acx / (float)4096);
	     printf("%f\r\n", acy / (float)4096);
	     printf("%f\r\n", acz / (float)4096);

	     HAL_UART_Transmit(&huart2, serialBuffer, strlen((char *)serialBuffer), 10);
//	     while ((HAL_GetTick() - _millis) < 4);
	     HAL_Delay(500);
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void MPU6050_Init(void) {
  uint8_t PWR_MGMT_1[2] = {0x6B, 0x00};
  while (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 2, 10) != HAL_OK);

  uint8_t GYR_CONFIG[2] = {0x1B, 0x08};
  while(HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, GYR_CONFIG, 2, 10) != HAL_OK);

  uint8_t ACC_CONFIG[2] = {0x1C, 0x10};
  while(HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, ACC_CONFIG, 2, 10) != HAL_OK);

  uint8_t LPF_CONFIG[2] = {0x1A, 0x03};
  while(HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, LPF_CONFIG, 2, 10) != HAL_OK);
}

/* MPU6050 read data function ------------------------------------------------*/
void read_MPU6050(void) {
  uint8_t data[14];
  uint8_t reg = MPU6050_ACCEL_XOUT_H;

  while(HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 1000) != HAL_OK);
  while(HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, data, 14, 1000) != HAL_OK);

  /* Formatted accelerometer data */
  acx = (int16_t)(data[0] << 8 | data[1]);
  acy = (int16_t)(data[2] << 8 | data[3]);
  acz = (int16_t)(data[4] << 8 | data[5]);

  /* Formatted temperature */
  tmp = (data[6] << 8 | data[7]);

  /* Formatted gyroscope data */
  gyx = (int16_t)(data[8] << 8 | data[9]);
  gyy = (int16_t)(data[10] << 8 | data[11]);
  gyz = (int16_t)(data[12] << 8 | data[13]);
}

/* MPU6050 calibrate function ------------------------------------------------*/
void calibirate_MPU6050(int num_iterations) {
  strcpy((char *)serialBuffer, "Calibrating MPU6050..\r\n");
  HAL_UART_Transmit(&huart2, serialBuffer, strlen((char *)serialBuffer), 10);
  for (int i = 0; i < num_iterations; i++) {
    read_MPU6050();
    offset_gyx += gyx;
    offset_gyy += gyy;
    offset_gyz += gyz;
  }

  offset_gyx /= num_iterations;
  offset_gyy /= num_iterations;
  offset_gyz /= num_iterations;

  strcpy((char *)serialBuffer, "Done calibrating!\r\n");
  HAL_UART_Transmit(&huart2, serialBuffer, strlen((char *)serialBuffer), 10);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
