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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
//#include "filter.h"
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
//float adc_buffer[SAMPLES_COUNT] = { 0 };
volatile uint8_t adc_ready_conversion = 0;
volatile uint32_t adc_buffer_counter = 0;
//volatile uint8_t adc_dma_ready = 0;
//float *adc_buffer = 0;
volatile static uint8_t start_flag = 0;
volatile uint8_t uart_wait_flag = 0;
//volatile uint16_t adc_dma_buffer[3]; // x-, y- distance sensor, CRR
//volatile uint32_t adc_buf_iterator = 0;
volatile uint16_t line_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//static void convert_buffers(float *in_data, char *out_data,
//		uint32_t input_buffer_size);
static void convert_buffers(uint16_t *in_data, uint8_t *out_data,
		uint32_t input_buffer_size);
//static void _float_to_char(float x, char *p, uint32_t string_buffer_size);
//static uint8_t _string_append(char *s1, char *s2, uint16_t s1_length,
//		uint16_t s2_length);
//static void _add_data_to_string(float data, char *out_data,
//		uint16_t out_data_length);
//static void _append_EOS(char *s1, uint16_t input_string_length,
//		uint8_t type_EOS);
//static void _append_divider(char *input, uint16_t length);

//static uint8_t _check_SD_status(FIL *file, char *file_name);
static uint8_t _write_data_SD(char *file_name, uint8_t *data,
		uint16_t length);

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
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	FATFS fs;

	uint8_t rx_foo[3];
	HAL_UART_Receive_IT(&huart2, rx_foo, 3);
	uint32_t measurements_counter = 0;
	f_mount(&fs, "", 0);
	//start_flag = 1;
	while (1) {
		if (start_flag) {
			start_flag = 0;
			measurements_counter = 0;
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			HAL_ADC_Start_IT(&hadc1);
			uint8_t rx_foo2[3];
			HAL_UART_Receive_IT(&huart2, rx_foo2, 3);


		}
		if (adc_ready_conversion) {
			adc_ready_conversion = 0;
			if (measurements_counter < FRAMES_COUNT) {
				measurements_counter++;
				uint8_t *tx_buffer = malloc(SAMPLES_COUNT * sizeof(uint16_t));
//				uint8_t tx_buffer[SAMPLES_COUNT * sizeof(uint16_t)];
//				uint8_t m = 5;
//				uint8_t degree = 3;
//				float temp_buff[m];
//				for (uint32_t k = 0; k < SAMPLES_COUNT; k++) {
//					for (uint32_t i = 0; i < m; i++)
//						temp_buff[i] = adc_buffer[k * m + i];
//					filter(temp_buff, degree, m);
//				}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
				convert_buffers((uint16_t*) adc_buffer, tx_buffer,
				SAMPLES_COUNT);
//				uint8_t ai_waiting_error_counter = 0;
//				uint8_t _ai_ready = aiGetReady();
//				while (!_ai_ready) {
//					HAL_Delay(1);
//					if (ai_waiting_error_counter > 20) {
//						Error_Handler();
//						break;
//					} else
//						ai_waiting_error_counter++;
//				}
//				float ai_result[2] = { -1 };
//				aiGetResult(ai_result, 2);
//				_add_data_to_string(ai_result[0], (char*) tx_buffer,
//						sizeof(tx_buffer));
//				_append_divider((char*) tx_buffer, sizeof(tx_buffer));
//				_add_data_to_string(ai_result[1], (char*) tx_buffer,
//						sizeof(tx_buffer));
//				_append_EOS((char*) tx_buffer, sizeof(tx_buffer), 2);
//				_append_EOS((char*) tx_buffer, sizeof(tx_buffer), 0); //for Matlab connection
//				f_printf(&file, (TCHAR*) tx_buffer);
				if(_write_data_SD(FILE_NAME, tx_buffer, SAMPLES_COUNT * sizeof(uint16_t)) != HAL_OK){
					Error_Handler();
				}

				uart_wait_flag = 1;
				HAL_UART_Transmit_IT(&huart2, tx_buffer,
				SAMPLES_COUNT * sizeof(uint16_t));
				if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
					Error_Handler();
				}
				uint8_t uart_error_counter = 0;
				while (uart_wait_flag) {
					HAL_Delay(1);
					if (uart_error_counter > 20) {
						Error_Handler();
						break;
					} else
						uart_error_counter++;
				}
				free(tx_buffer);
			} else {
				HAL_ADC_Stop_IT(&hadc1);
				measurements_counter = 0;
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

				//GET ELAPSED TIME OF ONE FRAME
//				HAL_TIM_Base_Stop(&htim6);
//				now_ticks = __HAL_TIM_GET_COUNTER(&htim6);
//				elapsed_time = now_ticks - start_ticks;
//				elapsed_time /= 1000000;
//				__HAL_TIM_SET_COUNTER(&htim6, 0);
//				HAL_TIM_Base_Start(&htim6);
//				start_ticks = __HAL_TIM_GET_COUNTER(&htim6);
				//GET ELAPSED TIME OF ONE FRAME
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_SDMMC1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static uint8_t _write_data_SD(char *file_name, uint8_t *data,
		uint16_t length) {
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

//static uint8_t _check_SD_status(FIL *file, char *file_name) {
//	uint8_t status = HAL_OK;
//	uint8_t error_counter = 0;
//
//	FRESULT fr;
//
//	fr = f_open(file, file_name, FA_OPEN_ALWAYS);
//	f_sync(file);
//	fr = f_close(file);
//	while (fr != FR_OK) {
//		HAL_Delay(1);
//
//		fr = f_open(file, file_name, FA_OPEN_ALWAYS);
//		fr = f_close(file);
//		error_counter++;
//		if (error_counter > 10) {
//			status = HAL_ERROR;
//			break;
//		}
//	}
//
//	return status;
//}

//static void convert_buffers(float *in_data, char *out_data,
//		uint32_t input_buffer_size) {
//	uint32_t last_pos = 0;
//	for (uint32_t i = 0; i < input_buffer_size; i++) {
//		char temp_string[10] = "";
//		_float_to_char(in_data[i], temp_string, sizeof(temp_string));
//
//		//append new data to output buffer
//		for (uint32_t j = 0; j < 10; j++) {
//			if (temp_string[j] != '\0') {
//				*(out_data + last_pos) = temp_string[j];
//				++last_pos;
//			}
//		}
//		out_data[last_pos] = STRING_VALUE_DIVIDER;
//		++last_pos;
//	}
//}
static void convert_buffers(uint16_t *in_data, uint8_t *out_data,
		uint32_t input_buffer_size) {
	for (uint32_t i = 0, j = 0; i < input_buffer_size; i++) {
		uint16_t temp = in_data[i];
		out_data[j] = temp >> 8;
		++j;
		out_data[j] = temp;
		++j;
//		uint8_t * temp = (uint8_t *)(&in_data[i]);
//		for (uint8_t j = sizeof(temp); j > 0; j--){
//			out_data[i] = temp[j];
//			i++;
//		}
	}
}

//static void _add_data_to_string(float data, char *out_data,
//		uint16_t out_data_length) {
//	const uint16_t string_length = 10;
//	char temp_string[10] = "";
//	_float_to_char(data, temp_string, string_length);
//	for (uint16_t i = 0; i < string_length; i++) {
//		if (temp_string[i] != '\0') {
//			for (uint16_t j = 0; i < string_length; j++, i++) {
//				temp_string[j] = temp_string[i];
//				temp_string[i] = '\0';
//			}
//			break;
//		}
//	}
//	_string_append(out_data, temp_string, out_data_length, strlen(temp_string));
//}

//static void _append_divider(char *input, uint16_t length) {
//	uint32_t iter = length - 1;
//	do {
//		if (input[iter] != '\0')
//			break;
//		else
//			iter--;
//	} while (iter > 0);
//	input[iter + 1] = STRING_VALUE_DIVIDER;
//}

//append string s2 to end of s1 string
//static uint8_t _string_append(char *s1, char *s2, uint16_t s1_length,
//		uint16_t s2_length) {
//
//	//find last symbol in string s1
//	uint32_t iter = s1_length - 1;
//	do {
//		if (s1[iter] != '\0')
//			break;
//		else
//			iter--;
//	} while (iter > 0);
//
//	//if s2 more than s1 can contain then return error
//	if ((s2_length + (iter + 1)) > s1_length)
//		return 1;
//
//	for (uint32_t i = iter + 1, j = 0; j < s2_length; i++, j++) {
//		s1[i] = s2[j];
//	}
//	return 0;
//}

// types: 1 - '\r', 2 - '\n', other - "\r\n"
//static void _append_EOS(char *s1, uint16_t input_string_length,
//		uint8_t type_EOS) {
//	char EOS_strin[2] = "";
//	switch (type_EOS) {
//	case 1:
//		EOS_strin[0] = '\r';
//		break;
//	case 2:
//		EOS_strin[0] = '\n';
//		break;
//	default:
//		EOS_strin[0] = '\r';
//		EOS_strin[1] = '\n';
//		break;
//	}
//	_string_append(s1, EOS_strin, input_string_length, 2);
//}

//static void _float_to_char(float x, char *p, uint32_t string_buffer_size) {
//	char *s = p + string_buffer_size; // go to end of buffer
//	uint16_t decimals;  // variable to store the decimals
//	int units; // variable to store the units (part to left of decimal place)
//	if (x < 0) { // take care of negative numbers
//		decimals = (int) (x * -1000) % 1000; // make 1000 for 3 decimals etc.
//		units = (int) (-1 * x);
//	} else { // positive numbers
//		decimals = (int) (x * 1000) % 1000;
//		units = (int) x;
//	}
//
//	do {
//		*--s = (decimals % 10) + '0';
//		decimals /= 10; // repeat for as many decimal places as you need
//	} while (decimals > 0);
//	*--s = '.';
//	do {
//		*--s = (units % 10) + '0';
//		units /= 10;
//	} while (units > 0);
//
//	if (x < 0)
//		*--s = '-'; // unary minus sign for negative numbers
//	p = s;
//}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == hadc1.Instance) {
			if (adc_buffer_counter < SAMPLES_COUNT) {
				adc_buffer[adc_buffer_counter] = HAL_ADC_GetValue(&hadc1);// * ADC_CONVERSION;
				adc_buffer_counter++;
				if ((adc_buffer_counter) % 3 == 0) {
					HAL_ADC_Stop_IT(&hadc1);
					HAL_ADC_Start_IT(&hadc3);

				} else {
					HAL_ADC_Start_IT(&hadc1);
				}
			} else {
				adc_ready_conversion = 1;
				adc_buffer_counter = 0;
			}
		}
		if (hadc->Instance == hadc3.Instance) {
			if (adc_buffer_counter < SAMPLES_COUNT) {
				adc_buffer[adc_buffer_counter] = HAL_ADC_GetValue(&hadc3);// * ADC_CONVERSION;
				adc_buffer_counter++;
				if ((adc_buffer_counter) % 3 == 0) {
					HAL_ADC_Stop_IT(&hadc3);
					HAL_ADC_Start_IT(&hadc1);
				}
			} else {
				adc_ready_conversion = 1;
				adc_buffer_counter = 0;
			}
		}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart2.Instance) {
		uart_wait_flag = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart2.Instance) {
		start_flag = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == USER_Btn_Pin) {
		start_flag = 1;
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
