/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void Tmp102Init(void);	// Initialization sequence for TMP102 sensor
void LogMessage(int8_t* msgStr);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint16_t tmp102Address = 0x48 << 1;	// required left-shift by 1 by the HAL_I2C_IsDeviceReady
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LogMessage("Initializing I2C...\r\n");
  HAL_I2C_MspInit(&hi2c2);	// initializes the GPIO I2C pins (SDA and SCL), initializes I2C clock
  LogMessage("I2C initialization complete\r\n");

  LogMessage("Initializing TMP102 sensor...\r\n");
  Tmp102Init();
  LogMessage("TMP102 sensor initialization complete\r\n");






  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Tmp102Init()
{

	  uint32_t trials = 1;
	  uint32_t timeout = 1000; // milliseconds
	  // TODO figure out why changing the tmp102 address to other available addresses doesn't work.
	  // TODO figure out why we had to left-shift the address by 1
	  if(HAL_I2C_IsDeviceReady(&hi2c2, tmp102Address, trials, timeout) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  else
	  {
		   LogMessage("TMP102 sensor ACK received\r\n");
	  }

	  // Check if TMP102 is in the default configuration ("Section 7.5.3 Configuration Register" of TMP102 Datasheet)
	  const uint16_t defaultConfigMSB = 0x60;
	  const uint16_t defaultConfigLSB = 0xA0;
//	  const uint32_t defaultConfigVal = (uint32_t) defaultConfigMSB + (uint32_t) defaultConfigLSB;

	  // TODO Study I2C_TransferConfig method
	  uint8_t readBuffer[2] = {'\0'};
	  uint16_t registerPtr = 0x01; // tmp102's pointer register value of 0x01 indicates access to the configuration register

	  // set the register pointer's value to the register wanted to be read
	  if(HAL_I2C_Master_Transmit(&hi2c2, tmp102Address, &registerPtr, sizeof(uint8_t), timeout) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  else
	  {
		  LogMessage("TMP102 Register Pointer write success\r\n");
	  }

	  // receive the 2 x 8bit (2 bytes) data into the readBuffer
	  if(HAL_I2C_Master_Receive(&hi2c2, tmp102Address, readBuffer, sizeof(uint16_t), timeout) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  else
	  {
		  if(readBuffer[0] == defaultConfigMSB && readBuffer[1] == defaultConfigLSB)
		  {
			  LogMessage("TMP102 Configuration Register in default configuration\r\n");
		  }
		  else
		  {
			  Error_Handler();
		  }
	  }

//	  if(HAL_I2C_Master_Receive(&hi2c2, tmp102Address, readBuffer, sizeof(readData), timeout) != HAL_OK)
//	  {
//	  } // TODO change to non-blocking mode when we are more familiar with I2C.
//	  else
//	  {
//		  uint8_t* pReadData = &readBuffer;
//		  if(*pReadData != defaultConfigVal)
//		  {
//			  LogMessage("Config error \r\n");
//		  }
//	  }



}

/*
 *  Displays contents of msgStr to console
 *  NOTE: Uses USART1
 */
void LogMessage(int8_t* msgStr)
{
	  const int timeoutMs = 100;
	  HAL_UART_Transmit(&huart1, msgStr, strlen(msgStr), timeoutMs);
};

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

  char errMsg[] = "Error encountered. Terminate the program\r\n";
  HAL_UART_Transmit(&huart1, errMsg, sizeof(errMsg), HAL_MAX_DELAY);

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
