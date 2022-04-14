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
#include "mpu6050.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
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

/* USER CODE BEGIN PV */
int16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
float ax, ay, az, gx, gy, gz, temperature, roll, pitch;
uint8_t buffer[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART2_SendString(char* s)
{
 HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init(&hi2c1);

   MPU6050_SetInterruptMode(MPU6050_INTMODE_ACTIVEHIGH);
   MPU6050_SetInterruptDrive(MPU6050_INTDRV_PUSHPULL);
   MPU6050_SetInterruptLatch(MPU6050_INTLATCH_WAITCLEAR);
   MPU6050_SetInterruptLatchClear(MPU6050_INTCLEAR_STATUSREAD);

   MPU6050_SetIntEnableRegister(0); // Disable all interrupts

   // Enable Motion interrputs
   MPU6050_SetDHPFMode(MPU6050_DHPF_5);

   MPU6050_SetIntMotionEnabled(1);
   MPU6050_SetIntZeroMotionEnabled(1);
   MPU6050_SetIntFreeFallEnabled(1);

   MPU6050_SetFreeFallDetectionDuration(2);
   MPU6050_SetFreeFallDetectionThreshold(5);

   MPU6050_SetMotionDetectionDuration(5);
   MPU6050_SetMotionDetectionThreshold(2);

   MPU6050_SetZeroMotionDetectionDuration(2);
   MPU6050_SetZeroMotionDetectionThreshold(4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	  MPU6050_GetAccelerometerScaled(&ax, &ay, &az);
	 	  MPU6050_GetGyroscopeScaled(&gx, &gy, &gz);
	 	  temperature = MPU6050_GetTemperatureCelsius();
//	 	  MPU6050_GetAccelerationXRAW();
//	 	  MPU6050_GetAccelerationYRAW();
//	 	  MPU6050_GetAccelerationZRAW();
	 	  memset(buffer, 0, 128);
	 	  sprintf((char*)buffer, "ACC: X: %.2f Y:%.2f Z:%.2f \n\rGYR: X: %.2f Y:%.2f Z:%.2f\n\rTEMP: %.2f\n\r", ax, ay, az, gx, gy, gz, temperature);
	 	  UART2_SendString((char*)buffer);

	 	  MPU6050_GetRollPitch(&roll, &pitch);
	 	  memset(buffer, 0, 128);
	 	  sprintf((char*)buffer, "RPY: Roll: %.2f Pitch: %.2f\n\r\n\r", roll, pitch);
	 	  UART2_SendString((char*)buffer);
	 	  HAL_Delay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == INT_Pin)
	{
		uint8_t interrupts = MPU6050_GetIntStatusRegister();
		MPU6050_GetMotionStatusRegister();
		sprintf((char*)buffer, "Int status triggered: %X\n\n\r", interrupts);
		UART2_SendString((char*)buffer);

		if(interrupts & (1<<MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) // Bit 4 (0x10)
		{
			sprintf((char*)buffer, "FIFO Overflow detected\n\r");
			UART2_SendString((char*)buffer);
		}

		if(interrupts & (1<<MPU6050_INTERRUPT_MOT_BIT))	// Bit 6 (0x40)
		{
			sprintf((char*)buffer, "Motion detected\n\r");
			UART2_SendString((char*)buffer);
		}

		if(interrupts & (1<<MPU6050_INTERRUPT_ZMOT_BIT))	// Bit 5 (0x20)
		{
			sprintf((char*)buffer, "Zero Motion detected\n\r");
			UART2_SendString((char*)buffer);
		}

		if(interrupts & (1<<MPU6050_INTERRUPT_FF_BIT))	// Bit 7 (0x80)
		{
			sprintf((char*)buffer, "Freefall detected\n\r");
			UART2_SendString((char*)buffer);
		}

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
