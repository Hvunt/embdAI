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
#include "adc.h"
#include "crc.h"
#include "fatfs.h"
#include "sdmmc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "usbd_cdc_if.h"
//#include "filter.h"
#include "device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILE_NAME "LOG.BIN"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define STRING_VALUE_DIVIDER ('_')

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t adc_buffer[SAMPLES_COUNT] = { 0 };
volatile uint8_t adc_ready_conversion = 0;
volatile uint32_t adc_buffer_counter = 0;
volatile uint8_t adc_module_counter = 0;

volatile static uint8_t start_flag = 0;
volatile uint16_t line_counter = 0;

volatile uint8_t new_data_flag = 0;

extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

volatile Device_t device;
volatile DeviceSettings_t deviceSettings;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void convert_buffers(uint16_t *in_data, uint8_t *out_data,
		uint32_t input_buffer_size);
static uint8_t _write_data_SD(char *file_name, uint8_t *data, uint16_t length);
static void _us_delay(uint32_t delay);

static void _read_adc();
static void _clear_adc_buffer();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_ADC1_Init();
	MX_CRC_Init();
	MX_SDMMC1_SD_Init();
	MX_FATFS_Init();
	MX_ADC3_Init();
	MX_TIM6_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	FATFS fs;
	uint32_t measurements_counter = 0;
	f_mount(&fs, "", 0);

	//default settings for device
	device.action = ACTION_STOP;
	deviceSettings.sd_card_record = SD_CARD_RECORD_ALL;
	deviceSettings.dis_sens_count = 4;
	deviceSettings.t_sens_count = 2;
	deviceSettings.time_interval = 100;

	while (1) {
		if (new_data_flag) {
			new_data_flag = 0;
			switch (device.action) {
			case ACTION_RUN:
				start_flag = 1;
				break;
			case ACTION_STOP:
				start_flag = 0;
				break;
			case ACTION_GET:
				if (device.sub_action == ACTION_DATA) {
					start_flag = 1;
				} else if (device.sub_action == ACTION_SETTINGS) {
					uint16_t data = getSetting(
							(DeviceSettings_t*) &deviceSettings,
							device.setting);
					uint8_t temp[2] = { };
					temp[0] = data >> 8;
					temp[1] = data;
					CDC_Transmit_FS(temp, sizeof(temp));
				}
				break;
			case ACTION_SET:

				setSettings((DeviceSettings_t*) &deviceSettings, device.setting,
						device.data);
				break;
			}
		}
		if (start_flag) {
			if ((device.action == ACTION_GET)
					&& (device.sub_action == ACTION_DATA))
				start_flag = 0;
//			measurements_counter = 0;
//			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//			HAL_ADC_Start_IT(&hadc1);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			_read_adc();
			uint8_t *tx_buffer = malloc(SAMPLES_COUNT * sizeof(uint16_t));

			convert_buffers((uint16_t*) adc_buffer, tx_buffer,
			SAMPLES_COUNT);

			if (CDC_Transmit_FS(tx_buffer, SAMPLES_COUNT * sizeof(uint16_t))
					!= USBD_OK) {
				Error_Handler();
			}
			if ((deviceSettings.sd_card_record == SD_CARD_RECORD_ALL)
					|| (deviceSettings.sd_card_record == SD_CARD_RECORD_GET)) {
				if (_write_data_SD(FILE_NAME, tx_buffer,
				SAMPLES_COUNT * sizeof(uint16_t)) != HAL_OK) {
					Error_Handler();
				}
			}

			free(tx_buffer);
			_clear_adc_buffer();

			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}
		if (adc_ready_conversion) {
//			adc_ready_conversion = 0;
//			if (measurements_counter < FRAMES_COUNT) {
//				measurements_counter++;
//				uint8_t *tx_buffer = malloc(SAMPLES_COUNT * sizeof(uint16_t));

			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
//				convert_buffers((uint16_t*) adc_buffer, tx_buffer,
//				SAMPLES_COUNT);
//
//				if (CDC_Transmit_FS(tx_buffer, SAMPLES_COUNT * sizeof(uint16_t))
//						!= USBD_OK) {
//					Error_Handler();
//				}
//
//				if ((deviceSettings.sd_card_record == SD_CARD_RECORD_ALL)
//						|| (deviceSettings.sd_card_record == SD_CARD_RECORD_GET)) {
//					if (_write_data_SD(FILE_NAME, tx_buffer,
//					SAMPLES_COUNT * sizeof(uint16_t)) != HAL_OK) {
//						Error_Handler();
//					}
//				}
//
//				free(tx_buffer);
//				_clear_adc_buffer();
//				ersion) {
//				//			adc_ready_conversion = 0;
//				//			if (measurements_counter < FRAMES_COUNT) {
//				//				measurements_counter++;
//				//				uint8_t *t
//				if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
//					Error_Handler();
//				}
//
//			} else {
//				HAL_ADC_Stop_IT(&hadc1);
//				HAL_ADC_Stop_IT(&hadc3);
//				measurements_counter = 0;
//				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//			}
		}
		_us_delay(deviceSettings.time_interval);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1
			| RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

static void _read_adc() {
	for (uint32_t i = 0; i < SAMPLES_COUNT;) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		adc_buffer[i] = HAL_ADC_GetValue(&hadc1);
		i++;
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		adc_buffer[i] = HAL_ADC_GetValue(&hadc1);
		i++;
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		adc_buffer[i] = HAL_ADC_GetValue(&hadc1);
		i++;
		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3, 10);
		adc_buffer[i] = HAL_ADC_GetValue(&hadc3);
		i++;
		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3, 10);
		adc_buffer[i] = HAL_ADC_GetValue(&hadc3);
		i++;
		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3, 10);
		adc_buffer[i] = HAL_ADC_GetValue(&hadc3);
		i++;
	}
}

static void _clear_adc_buffer() {
	for (uint32_t i = 0; i < SAMPLES_COUNT; i++) {
		adc_buffer[i] = 0;
	}
}

static void _us_delay(uint32_t delay) {
	TIM6->SR = 0;
	TIM6->ARR = delay;
	TIM6->PSC = 108 - 1;
	TIM6->CR1 |= TIM_CR1_CEN;
	while (!(TIM6->SR & TIM_SR_UIF))
		;
}

static uint8_t _write_data_SD(char *file_name, uint8_t *data, uint16_t length) {
	FIL file;
	FRESULT fr;
	UINT bw;

	fr = f_open(&file, file_name, FA_WRITE | FA_OPEN_APPEND);
	f_sync(&file);
	if (fr == FR_OK) {
		//seek to end of the file to append data
		fr = f_lseek(&file, f_size(&file));
		if (fr != FR_OK) {
			f_close(&file);
			return HAL_ERROR;
		}
	} else {
		f_close(&file);
		return HAL_ERROR;
	}

	uint8_t temp[2] = { line_counter >> 8, line_counter };
	fr = f_write(&file, temp, sizeof(temp), &bw);
	if (fr != FR_OK) {
		f_close(&file);
		return HAL_ERROR;
	}
	f_sync(&file);
	line_counter++;

	fr = f_write(&file, data, length, &bw);
	if (fr != FR_OK) {
		f_close(&file);
		return HAL_ERROR;
	}
	f_sync(&file);
	HAL_Delay(1);

	uint16_t divider = 0xFFFF;
	fr = f_write(&file, &divider, sizeof(divider), &bw);
	if (fr != FR_OK) {
		f_close(&file);
		return HAL_ERROR;
	}
	f_sync(&file);

	f_close(&file);
	return HAL_OK;
}

static void convert_buffers(uint16_t *in_data, uint8_t *out_data,
		uint32_t input_buffer_size) {
	for (uint32_t i = 0, j = 0; i < input_buffer_size; i++) {
		uint16_t temp = in_data[i];
		out_data[j] = temp >> 8;
		++j;
		out_data[j] = temp;
		++j;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	if (hadc->Instance == hadc1.Instance) {
//		if (adc_buffer_counter < SAMPLES_COUNT) {
//			adc_buffer[adc_buffer_counter] = HAL_ADC_GetValue(&hadc1); // * ADC_CONVERSION;
//			adc_buffer_counter++;
//			if (adc_module_counter > 3) {
//				adc_module_counter = 0;
//				HAL_ADC_Stop_IT(&hadc1);
//				HAL_ADC_Start_IT(&hadc3);
//
//			} else {
//				adc_module_counter++;
//				HAL_ADC_Start_IT(&hadc1);
//			}
//
//		} /*else {
//			adc_ready_conversion = 1;
//			adc_buffer_counter = 0;
//		}*/
//	} else if (hadc->Instance == hadc3.Instance) {
//		if (adc_buffer_counter < SAMPLES_COUNT) {
//			adc_buffer[adc_buffer_counter] = HAL_ADC_GetValue(&hadc3); // * ADC_CONVERSION;
//			adc_buffer_counter++;
////			if ((adc_buffer_counter) % 2 == 0) {
////				HAL_ADC_Stop_IT(&hadc3);
////				HAL_ADC_Start_IT(&hadc1);
////			} else {
//				HAL_ADC_Start_IT(&hadc3);
////			}
//		} else {
//			adc_ready_conversion = 1;
//			adc_buffer_counter = 0;
//		}
//	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == USER_Btn_Pin) {
		new_data_flag = 1;
		if (device.action == ACTION_STOP)
			device.action = ACTION_RUN;
		else if (device.action == ACTION_RUN)
			device.action = ACTION_STOP;
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
