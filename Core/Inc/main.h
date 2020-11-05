/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//#define ADC_CONVERSION 0.000806
//#define SAMPLES_COUNT 180*6
//#define FRAMES_COUNT 1
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MQTT_OUTPUT_RINGBUF_SIZE 4096
#define MQTT_REQ_MAX_IN_FLIGHT 6
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define EID_DC_Pin GPIO_PIN_4
#define EID_DC_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define EID_RST_Pin GPIO_PIN_12
#define EID_RST_GPIO_Port GPIOF
#define SPI_TCS2_Pin GPIO_PIN_8
#define SPI_TCS2_GPIO_Port GPIOE
#define SPI_TCS1_Pin GPIO_PIN_15
#define SPI_TCS1_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define LED_STATUS_Pin GPIO_PIN_12
#define LED_STATUS_GPIO_Port GPIOD
#define LED_ERROR_Pin GPIO_PIN_13
#define LED_ERROR_GPIO_Port GPIOD
#define EID_CS_Pin GPIO_PIN_15
#define EID_CS_GPIO_Port GPIOD
#define SDMMC1_CD_Pin GPIO_PIN_2
#define SDMMC1_CD_GPIO_Port GPIOG
#define EID_CLK_Pin GPIO_PIN_3
#define EID_CLK_GPIO_Port GPIOB
#define EID_BUSY_Pin GPIO_PIN_4
#define EID_BUSY_GPIO_Port GPIOB
#define EID_DIN_Pin GPIO_PIN_5
#define EID_DIN_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define WIFI_RX_Pin GPIO_PIN_8
#define WIFI_RX_GPIO_Port GPIOB
#define WIFI_TX_Pin GPIO_PIN_9
#define WIFI_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
