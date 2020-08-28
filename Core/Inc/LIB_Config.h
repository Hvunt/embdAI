/**
  ******************************************************************************
  * @file    LIB_Config.h
  * @author  Waveshare Team
  * @version 
  * @date    13-October-2014
  * @brief     This file provides configurations for low layer hardware libraries.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _USE_LIB_CONFIG_H_
#define _USE_LIB_CONFIG_H_
//Macro Definition

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_gpio.h"
#include "LIB_Config.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------------*/
//SPI
#include "spi.h"

/*------------------------------------------------------------------------------------------------------*/
//SSD1306
#include "SSD1306.h"
#include "Fonts.h"

#define SH1106
//#define SSD1306

#define INTERFACE_4WIRE_SPI     //4-wire SPI 

#define SSD1306_DC_Pin GPIO_PIN_3
#define SSD1306_DC_GPIO_Port GPIOE
#define SSD1306_RES_Pin GPIO_PIN_8
#define SSD1306_RES_GPIO_Port GPIOF
#define SSD1306_CS_Pin GPIO_PIN_4
#define SSD1306_CS_GPIO_Port GPIOE

#define __SSD1306_CS_SET()      HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, GPIO_PIN_SET)
#define __SSD1306_CS_CLR()      HAL_GPIO_WritePin(SSD1306_CS_GPIO_Port, SSD1306_CS_Pin, GPIO_PIN_RESET)

#define __SSD1306_RES_SET()     HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_SET)
#define __SSD1306_RES_CLR()     HAL_GPIO_WritePin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin, GPIO_PIN_RESET)

#define __SSD1306_DC_SET()      HAL_GPIO_WritePin(SSD1306_RES_GPIO_Port, SSD1306_RES_Pin, GPIO_PIN_SET)
#define __SSD1306_DC_CLR()      HAL_GPIO_WritePin(SSD1306_RES_GPIO_Port, SSD1306_RES_Pin, GPIO_PIN_RESET)

//#define __SSD1306_CLK_SET()     GPIO_WriteBit(SSD1306_CLK_GPIO, SSD1306_CLK_PIN, Bit_SET)
//#define __SSD1306_CLK_CLR()     GPIO_WriteBit(SSD1306_CLK_GPIO, SSD1306_CLK_PIN, Bit_RESET)
//
//#define __SSD1306_DIN_SET()     GPIO_WriteBit(SSD1306_DIN_GPIO, SSD1306_DIN_PIN, Bit_SET)
//#define __SSD1306_DIN_CLR()     GPIO_WriteBit(SSD1306_DIN_GPIO, SSD1306_DIN_PIN, Bit_RESET)

//#define __SSD1306_CS_SET()      GPIO_WriteBit(SSD1306_CS_GPIO, SSD1306_CS_PIN, Bit_SET)
//#define __SSD1306_CS_CLR()      GPIO_WriteBit(SSD1306_CS_GPIO, SSD1306_CS_PIN, Bit_RESET)
//
//#define __SSD1306_RES_SET()     GPIO_WriteBit(SSD1306_RES_GPIO, SSD1306_RES_PIN, Bit_SET)
//#define __SSD1306_RES_CLR()     GPIO_WriteBit(SSD1306_RES_GPIO, SSD1306_RES_PIN, Bit_RESET)
//
//#define __SSD1306_DC_SET()      GPIO_WriteBit(SSD1306_DC_GPIO, SSD1306_DC_PIN, Bit_SET)
//#define __SSD1306_DC_CLR()      GPIO_WriteBit(SSD1306_DC_GPIO, SSD1306_DC_PIN, Bit_RESET)
//
//#define __SSD1306_CLK_SET()     GPIO_WriteBit(SSD1306_CLK_GPIO, SSD1306_CLK_PIN, Bit_SET)
//#define __SSD1306_CLK_CLR()     GPIO_WriteBit(SSD1306_CLK_GPIO, SSD1306_CLK_PIN, Bit_RESET)
//
//#define __SSD1306_DIN_SET()     GPIO_WriteBit(SSD1306_DIN_GPIO, SSD1306_DIN_PIN, Bit_SET)
//#define __SSD1306_DIN_CLR()     GPIO_WriteBit(SSD1306_DIN_GPIO, SSD1306_DIN_PIN, Bit_RESET)

#define __SSD1306_WRITE_BYTE(DATA) HAL_SPI_Transmit(&hspi4, DATA, 1, 0x1000)
/*------------------------------------------------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */


#endif

/*-------------------------------END OF FILE-------------------------------*/

