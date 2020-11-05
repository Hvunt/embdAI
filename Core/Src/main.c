/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "iwdg.h"
#include "mbedtls.h"
#include "rng.h"
#include "rtc.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <string.h>
//#include <math.h>
//#include "filter.h"
//#include "device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define FILE_NAME "LOG.BIN"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define STRING_VALUE_DIVIDER ('_')

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

//static void convert_buffers(uint16_t *in_data, uint8_t *out_data,
//		uint32_t input_buffer_size);
//static uint8_t _write_data_SD(char *file_name, uint8_t *data, uint16_t length);
//static void _us_delay(uint32_t delay);
//
//static void _read_adc();
//static void _clear_adc_buffer();
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DMA_Init();
  MX_CRC_Init();
  MX_SDMMC1_SD_Init();
  MX_ADC3_Init();
  MX_RNG_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_UART5_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	FATFS fs;
//	f_mount(&fs, "", 0);
//
//	//default settings for device
//	deviceAction.action = ACTION_STOP;
//	deviceSettingsInit((DeviceSettings_t *) &deviceSettings);

	while (1) {
//		if (new_data_flag) {
//			new_data_flag = 0;
//			switch (deviceAction.action) {
//			case ACTION_RUN:
//				start_flag = 1;
//				break;
//			case ACTION_STOP:
//				start_flag = 0;
//				break;
//			case ACTION_GET:
//				if (deviceAction.sub_action == ACTION_DATA) {
//					start_flag = 1;
//				} else if (deviceAction.sub_action == ACTION_SETTINGS) {
////					uint16_t data = deviceGetSetting(
////							(DeviceSettings_t*) &deviceSettings,
////							deviceAction.setting);
////					uint8_t temp[2] = { };
////					temp[0] = data >> 8;
////					temp[1] = data;
////					CDC_Transmit_FS(temp, sizeof(temp));
//				}
//				break;
//			case ACTION_SET:
//
//				deviceSetSettings((DeviceSettings_t*) &deviceSettings, deviceAction.setting,
//						deviceAction.data);
//				break;
//			}
//		}
//		if (start_flag) {
//			if ((deviceAction.action == ACTION_GET)
//					&& (deviceAction.sub_action == ACTION_DATA))
//				start_flag = 0;
//			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//			_read_adc();
////			uint8_t *tx_buffer = malloc(SAMPLES_COUNT * sizeof(uint16_t));
//			uint8_t tx_buffer[100];
//			convert_buffers((uint16_t*) adc_buffer, tx_buffer,
//			SAMPLES_COUNT);
//
////			if (CDC_Transmit_FS(tx_buffer, SAMPLES_COUNT * sizeof(uint16_t))
////					!= USBD_OK) {
////				Error_Handler();
////			}
//			if ((deviceSettings.sd_card_record == SD_CARD_RECORD_ALL)
//					|| (deviceSettings.sd_card_record == SD_CARD_RECORD_GET)) {
//				if (_write_data_SD(FILE_NAME, tx_buffer,
//				SAMPLES_COUNT * sizeof(uint16_t)) != HAL_OK) {
//					Error_Handler();
//				}
//			}
//
////			free(tx_buffer);
//			_clear_adc_buffer();
//
//			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//		}
//
//		_us_delay(deviceSettings.time_interval);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//static void _read_adc() {
//	for (uint32_t i = 0; i < SAMPLES_COUNT;) {
//		HAL_ADC_Start(&hadc3);
//		HAL_ADC_PollForConversion(&hadc3, 10);
//		adc_buffer[i] = HAL_ADC_GetValue(&hadc3);
//		i++;
//		HAL_ADC_Start(&hadc3);
//		HAL_ADC_PollForConversion(&hadc3, 10);
//		adc_buffer[i] = HAL_ADC_GetValue(&hadc3);
//		i++;
//		HAL_ADC_Start(&hadc3);
//		HAL_ADC_PollForConversion(&hadc3, 10);
//		adc_buffer[i] = HAL_ADC_GetValue(&hadc3);
//		i++;
//		HAL_ADC_Start(&hadc3);
//		HAL_ADC_PollForConversion(&hadc3, 10);
//		adc_buffer[i] = HAL_ADC_GetValue(&hadc3);
//		i++;
//		HAL_ADC_Start(&hadc3);
//		HAL_ADC_PollForConversion(&hadc3, 10);
//		adc_buffer[i] = HAL_ADC_GetValue(&hadc3);
//		i++;
//		HAL_ADC_Start(&hadc3);
//		HAL_ADC_PollForConversion(&hadc3, 10);
//		adc_buffer[i] = HAL_ADC_GetValue(&hadc3);
//		i++;
//	}
//}
//
//static void _clear_adc_buffer() {
//	for (uint32_t i = 0; i < SAMPLES_COUNT; i++) {
//		adc_buffer[i] = 0;
//	}
//}
//
//static void _us_delay(uint32_t delay) {
//	TIM6->SR = 0;
//	TIM6->ARR = delay;
//	TIM6->PSC = 108 - 1;
//	TIM6->CR1 |= TIM_CR1_CEN;
//	while (!(TIM6->SR & TIM_SR_UIF))
//		;
//}
//
//static uint8_t _write_data_SD(char *file_name, uint8_t *data, uint16_t length) {
//	FIL file;
//	FRESULT fr;
//	UINT bw;
//
//	fr = f_open(&file, file_name, FA_WRITE | FA_OPEN_APPEND);
//	f_sync(&file);
//	if (fr == FR_OK) {
//		//seek to end of the file to append data
//		fr = f_lseek(&file, f_size(&file));
//		if (fr != FR_OK) {
//			f_close(&file);
//			return HAL_ERROR;
//		}
//	} else {
//		f_close(&file);
//		return HAL_ERROR;
//	}
//
//	//add number of the samples
//	uint8_t temp[2] = { line_counter >> 8, line_counter };
//	fr = f_write(&file, temp, sizeof(temp), &bw);
//	if (fr != FR_OK) {
//		f_close(&file);
//		return HAL_ERROR;
//	}
//	f_sync(&file);
//	line_counter++;
//
//	//add samples data
//	fr = f_write(&file, data, length, &bw);
//	if (fr != FR_OK) {
//		f_close(&file);
//		return HAL_ERROR;
//	}
//	f_sync(&file);
//	HAL_Delay(1);
//
//	//add divider of the line
//	uint16_t divider = 0xFFFF;
//	fr = f_write(&file, &divider, sizeof(divider), &bw);
//	if (fr != FR_OK) {
//		f_close(&file);
//		return HAL_ERROR;
//	}
//	f_sync(&file);
//
//	f_close(&file);
//	return HAL_OK;
//}
//
//static void convert_buffers(uint16_t *in_data, uint8_t *out_data,
//		uint32_t input_buffer_size) {
//	for (uint32_t i = 0, j = 0; i < input_buffer_size; i++) {
//		uint16_t temp = in_data[i];
//		out_data[j] = temp >> 8;
//		++j;
//		out_data[j] = temp;
//		++j;
//	}
//}
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	if (GPIO_Pin == USER_Btn_Pin) {
//		new_data_flag = 1;
//		if (deviceAction.action == ACTION_STOP)
//			deviceAction.action = ACTION_RUN;
//		else if (deviceAction.action == ACTION_RUN)
//			deviceAction.action = ACTION_STOP;
//	}
//}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
