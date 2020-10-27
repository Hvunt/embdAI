/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

//#include "lwip/sys.h"
//#include "lwip/err.h"
#include "lwip.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "lwip/netif.h"
#include "lwip/apps/sntp.h"
#include "lwip/apps/mqtt.h"

#include "GUI_Paint.h"
#include "EPD_2in9bc.h"
#include "MAX31865.h"
#include "device.h"

#include "device.h"
#include "iwdg.h"
#include "rng.h"
#include "pepega.h"
#include "fatfs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
enum einkDrawDataTypes {
	EDDT_IP = 0,
	EDDT_CONNECTION_STATUS,
	EDDT_ERROR,
	EDDT_STATUS,
	EDDT_TIME,
	EDDT_UUID
};
typedef struct einkDrawData {
	uint8_t type;
	uint8_t *data;
	uint16_t length;
} einkDrawData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVER_ADDRESS 	"194.67.110.109"
#define SUB_TOPIC_NAME 	"devices/settings"

static const char PUB_TOPIC_NAME_POST_DATA[] = "/sensors";
static const char PUB_TOPIC_NAME_POST_SETTINGS[] = "/settings";
static const char PUB_TOPIC_NAME_PRED[] = "devices/";

#define DEVICE_LOGIN	"test_device"
#define DEVICE_PASSWORD "4556"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define clean_buff(buf, len) for (uint32_t i = 0; i < len;i++) buf[i] = 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//pool of memory for the eink display
UBYTE *BlackImage, *YellowImage;

extern RNG_HandleTypeDef hrng;
extern IWDG_HandleTypeDef hiwdg;
extern ADC_HandleTypeDef hadc3;

extern struct netif gnetif;

DeviceSettings_t device;
DeviceAction_t deviceState;
/* USER CODE END Variables */
/* Definitions for initTaskName */
osThreadId_t initTaskNameHandle;
const osThreadAttr_t initTaskName_attributes = {
  .name = "initTaskName",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for dataSend */
osThreadId_t dataSendHandle;
const osThreadAttr_t dataSend_attributes = {
  .name = "dataSend",
  .priority = (osPriority_t) osPriorityLow4,
  .stack_size = 300 * 4
};
/* Definitions for collectData */
osThreadId_t collectDataHandle;
const osThreadAttr_t collectData_attributes = {
  .name = "collectData",
  .priority = (osPriority_t) osPriorityLow3,
  .stack_size = 128 * 4
};
/* Definitions for logData */
osThreadId_t logDataHandle;
const osThreadAttr_t logData_attributes = {
  .name = "logData",
  .priority = (osPriority_t) osPriorityLow3,
  .stack_size = 128 * 4
};
/* Definitions for logStatus */
osThreadId_t logStatusHandle;
const osThreadAttr_t logStatus_attributes = {
  .name = "logStatus",
  .priority = (osPriority_t) osPriorityLow3,
  .stack_size = 1024 * 4
};
/* Definitions for sensorsData */
osMessageQueueId_t sensorsDataHandle;
uint8_t sensorsDataBuffer[ 210 * sizeof( uint8_t ) ];
osStaticMessageQDef_t sensorsDataControlBlock;
const osMessageQueueAttr_t sensorsData_attributes = {
  .name = "sensorsData",
  .cb_mem = &sensorsDataControlBlock,
  .cb_size = sizeof(sensorsDataControlBlock),
  .mq_mem = &sensorsDataBuffer,
  .mq_size = sizeof(sensorsDataBuffer)
};
/* Definitions for screenRefreshTim */
osTimerId_t screenRefreshTimHandle;
const osTimerAttr_t screenRefreshTim_attributes = {
  .name = "screenRefreshTim"
};
/* Definitions for dataBusyMutex */
osMutexId_t dataBusyMutexHandle;
const osMutexAttr_t dataBusyMutex_attributes = {
  .name = "dataBusyMutex"
};
/* Definitions for screenBusyMutex */
osMutexId_t screenBusyMutexHandle;
const osMutexAttr_t screenBusyMutex_attributes = {
  .name = "screenBusyMutex"
};
/* Definitions for sdBusyMutex */
osMutexId_t sdBusyMutexHandle;
const osMutexAttr_t sdBusyMutex_attributes = {
  .name = "sdBusyMutex"
};
/* Definitions for dataProcessingSem */
osSemaphoreId_t dataProcessingSemHandle;
const osSemaphoreAttr_t dataProcessingSem_attributes = {
  .name = "dataProcessingSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void screenInitTask(void *args);
void watchdogTask(void *args);
//void writeDataToSDTask(void *arg);
//
void drawMessageTask(void *message);

//MQTT connection
void connect_to_server(mqtt_client_t *client);
void serverFound(const char *name, const ip_addr_t *ip_addr, void *arg);
static void mqtt_connection_cb(mqtt_client_t *client, void *arg,
		mqtt_connection_status_t status);
static void mqtt_incoming_data_cb(void *arg, const uint8_t *data, uint16_t len,
		uint8_t flags);
static void mqtt_incoming_publish_cb(void *arg, const char *topic,
		u32_t tot_len);
static void mqtt_pub_request_cb(void *arg, err_t result);
static void mqtt_sub_request_cb(void *arg, err_t result);

//utility
static void getDeviceID(char *name);
static void makePubTopicName(char *out, size_t out_len);
//static void sendDataBuffClear(void);
//eink-display functions prototype
uint8_t screen_init();

/* USER CODE END FunctionPrototypes */

void initTask(void *argument);
void dataSendTask(void *argument);
void collectDataTask(void *argument);
void logDataTask(void *argument);
void logStatusTask(void *argument);
void screenRefreshCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void) {

}

__weak unsigned long getRunTimeCounterValue(void) {
	return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	char deviceID[12] = { 0 };
	getDeviceID(deviceID);
	deviceSettingsInit(&device);
	deviceSetID(&device, deviceID, sizeof(deviceID));
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of dataBusyMutex */
  dataBusyMutexHandle = osMutexNew(&dataBusyMutex_attributes);

  /* creation of screenBusyMutex */
  screenBusyMutexHandle = osMutexNew(&screenBusyMutex_attributes);

  /* creation of sdBusyMutex */
  sdBusyMutexHandle = osMutexNew(&sdBusyMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of dataProcessingSem */
  dataProcessingSemHandle = osSemaphoreNew(2, 2, &dataProcessingSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of screenRefreshTim */
  screenRefreshTimHandle = osTimerNew(screenRefreshCallback, osTimerPeriodic, NULL, &screenRefreshTim_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of sensorsData */
  sensorsDataHandle = osMessageQueueNew (210, sizeof(uint8_t), &sensorsData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of initTaskName */
//  initTaskNameHandle = osThreadNew(initTask, NULL, &initTaskName_attributes);

  /* creation of dataSend */
//  dataSendHandle = osThreadNew(dataSendTask, NULL, &dataSend_attributes);

  /* creation of collectData */
//  collectDataHandle = osThreadNew(collectDataTask, NULL, &collectData_attributes);

  /* creation of logData */
//  logDataHandle = osThreadNew(logDataTask, NULL, &logData_attributes);

  /* creation of logStatus */
  logStatusHandle = osThreadNew(logStatusTask, NULL, &logStatus_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//	const osThreadAttr_t screenInit_attributes = { .name = "screenInit",
//			.priority = (osPriority_t) osPriorityLow2, .stack_size = 256 * 4 };
//	osThreadNew(screenInitTask, NULL, &screenInit_attributes);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_initTask */
/**
 * @brief  Initialization of software part of the device.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_initTask */
void initTask(void *argument)
{
  /* USER CODE BEGIN initTask */
//	f_mount(&SDFatFS, "", 0);
//	ff_del_syncobj(&SDFatFS.sobj);
//	SDFatFS.sobj = DMA2BusySemHandle;

	osThreadExit();
  /* USER CODE END initTask */
}

/* USER CODE BEGIN Header_dataSendTask */
/**
 * @brief Function implementing the dataSend thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_dataSendTask */
void dataSendTask(void *argument)
{
  /* USER CODE BEGIN dataSendTask */
//	size_t freeHeapSize = xPortGetFreeHeapSize();
	MX_LWIP_Init();

	while (gnetif.ip_addr.addr == 0) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		osDelay(500);
	}
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	mqtt_client_t *client = mqtt_client_new();
	if (client != NULL) {
		connect_to_server(client);
	}

	//make topic name: device/[UUID]/sensor
	size_t pub_top_name_len = sizeof(device.deviceID) - 2
			+ sizeof(PUB_TOPIC_NAME_PRED) + sizeof(PUB_TOPIC_NAME_POST_DATA);
	char pub_top_name[pub_top_name_len];
	makePubTopicName(pub_top_name, pub_top_name_len);

	uint16_t sending_errors = 0;

	device.device_status = DEVICE_STATUS_AWAITING;
	for (;;) {
		if (deviceState.action == ACTION_RUN) {
			if (mqtt_client_is_connected(client)) {
				if (dataBusyMutexHandle != NULL) {
					osStatus_t result = osMutexAcquire(dataBusyMutexHandle,
					osWaitForever);
					if (result == osOK) {
						err_t err = mqtt_publish(client, pub_top_name,
								(uint8_t*) sensorsDataBuffer,
								sensorsData_attributes.mq_size, 0, 0,
								mqtt_pub_request_cb, NULL);
						if (err != ERR_OK) {
							HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,
									GPIO_PIN_SET);
							sending_errors++;
						} else {
							HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,
									GPIO_PIN_RESET);
						}
						osMutexRelease(dataBusyMutexHandle);
					}

				} else {
					HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
				}
			} else {
				osDelay(2000);
				connect_to_server(client);
			}
		}

		osDelay(100);
//		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
//		freeHeapSize = xPortGetFreeHeapSize();
	}
  /* USER CODE END dataSendTask */
}

/* USER CODE BEGIN Header_collectDataTask */
/**
 * @brief Function implementing the collectData thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_collectDataTask */
void collectDataTask(void *argument)
{
  /* USER CODE BEGIN collectDataTask */
//	Max31865_t max_1, max_2;
//	float temp1 = 0;
//	float temp2 = 0;
//	Max31865_init(&max_1, &hspi4, SPI_TCS1_GPIO_Port, SPI_TCS1_Pin, 2, 50);
//	Max31865_init(&max_2, &hspi4, SPI_TCS2_GPIO_Port, SPI_TCS2_Pin, 2, 50);
	for (;;) {
		if (dataBusyMutexHandle != NULL) {
			osStatus_t res = osMutexAcquire(dataBusyMutexHandle, osWaitForever);
			if (res == osOK) {
				clean_buff(sensorsDataBuffer, sensorsData_attributes.mq_size);
				for (uint16_t i = 0; i < sensorsData_attributes.mq_size;) {
//					uint16_t data[7] = { 0 };
//					osSemaphoreAcquire(adcReadySemHandle, osWaitForever); // does it need?

//					HAL_ADC_Start_DMA(&hadc3, (uint32_t*) data, 7);

					//collect data from MAX31865
//					Max31865_readTempC(&max_1, &temp1);
//					Max31865_readTempC(&max_2, &temp2);

//					osSemaphoreAcquire(adcReadySemHandle, osWaitForever);
					for (uint8_t j = 0; j < 7; j++, i += 2) {
						HAL_ADC_Start(&hadc3);
						HAL_ADC_PollForConversion(&hadc3, 10);
						uint16_t value = HAL_ADC_GetValue(&hadc3);
						sensorsDataBuffer[i] = value >> 8;
						sensorsDataBuffer[i + 1] = value;
					}
//					osSemaphoreRelease(adcReadySemHandle);
				}

				osMutexRelease(dataBusyMutexHandle);
			}
		}
	}
  /* USER CODE END collectDataTask */
}

/* USER CODE BEGIN Header_logDataTask */
/**
 * @brief Function implementing the logData thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_logDataTask */
void logDataTask(void *argument)
{
  /* USER CODE BEGIN logDataTask */
	/* Infinite loop */
	for (;;) {
//		if (dataBusyMutexHandle != NULL) {
//			osStatus_t res = osMutexAcquire(dataBusyMutexHandle, osWaitForever);
//			if (res == osOk) {
////				osSemaphoreAcquire(semaphore_id, timeout);
//
//			}
//		}
		osDelay(1000);
	}
  /* USER CODE END logDataTask */
}

/* USER CODE BEGIN Header_logStatusTask */
/**
 * @brief Function implementing the logStatus thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_logStatusTask */
void logStatusTask(void *argument)
{
  /* USER CODE BEGIN logStatusTask */
	char log_info[255];
	FIL logFile;
	FRESULT f_res;
	UINT bytes_written;
	if(f_mount(&SDFatFS, "", 0) != FR_OK){
		while(1){
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			osDelay(100);
		}
	}
	for (;;) {
		if (sdBusyMutexHandle != NULL) {
			osStatus_t res = osSemaphoreAcquire(sdBusyMutexHandle, osWaitForever);
			if (res == osOK) {
				char id[13] = { 0 };
				deviceGetID(&device, id, sizeof(id));
				sprintf(log_info, "ID: %s\n", id);
				if (f_open(&logFile, "LOG.LOG", FA_OPEN_APPEND | FA_WRITE)
						== FR_OK) {
					f_sync(&logFile);
					f_res = f_write(&logFile, log_info, strlen(log_info),
							&bytes_written);
					if (f_res != FR_OK) {
						HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
					}
					f_sync(&logFile);
					f_close(&logFile);
				}
				osSemaphoreRelease(sdBusyMutexHandle);
			}
		}
		osDelay(60000);
	}
  /* USER CODE END logStatusTask */
}

/* screenRefreshCallback function */
void screenRefreshCallback(void *argument)
{
  /* USER CODE BEGIN screenRefreshCallback */
	if (screenBusyMutexHandle != NULL) {
		osStatus_t res = osMutexAcquire(screenBusyMutexHandle, osWaitForever);
		if (res == osOK) {
			Paint_SelectImage(YellowImage);

			Paint_DrawBitMap(gImage_pepega);
			char data[40] = { 0 };

			Paint_DrawLine(1, 1, 100, 0, BLACK, DOT_PIXEL_1X1,
					LINE_STYLE_SOLID);
			sprintf(data, "Status code: %d", device.device_status);
			Paint_DrawString_EN(5, 16, data, &Font16, BLACK,
			WHITE);

			clean_buff(data, sizeof(data));
			char id[13] = { 0 };
			deviceGetID(&device, id, sizeof(id));
			sprintf(data, "My id: ");
			strcat(data, id);
			Paint_DrawString_EN(5, 35, data, &Font16, BLACK,
			WHITE);

			clean_buff(data, sizeof(data));
			sprintf(data, "My local IP: ");
			strcat(data, ip4addr_ntoa((const ip4_addr_t*) &gnetif.ip_addr));
			Paint_DrawString_EN(5, 52, data, &Font16, BLACK,
			WHITE);

			EPD_2IN9BC_Display(BlackImage, YellowImage);
			osMutexRelease(screenBusyMutexHandle);
		}
	}
  /* USER CODE END screenRefreshCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void watchdogTask(void *args) {
//	MX_IWDG_Init();
	for (;;) {
		osDelay(490);
//		HAL_IWDG_Refresh(&hiwdg);
	}
}
//
//void writeDataToSDTask(void *arg) {
//	for (;;) {
//
//	}
//}
//
//void collectDataTask(void *arg) {
//	for (;;) {
//
//	}
//}
//########### MQTT section #####
void connect_to_server(mqtt_client_t *client) {
	ip_addr_t server_addr;
	struct mqtt_connect_client_info_t ci;
	err_t err;

	memset(&ci, 0, sizeof(ci));

	char deviceID[13];
	deviceGetID(&device, deviceID, 13);
	ci.client_id = deviceID;
	ci.client_user = DEVICE_LOGIN;
	ci.client_pass = DEVICE_PASSWORD;
	ip4addr_aton(SERVER_ADDRESS, &server_addr);
	err = mqtt_client_connect(client, &server_addr, MQTT_PORT,
			mqtt_connection_cb,
			NULL, &ci);
	if (err != ERR_OK) {
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	} else
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg,
		mqtt_connection_status_t status) {
	err_t err;
	if (status == MQTT_CONNECT_ACCEPTED) {
		mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb,
				mqtt_incoming_data_cb, arg);
		err = mqtt_subscribe(client, SUB_TOPIC_NAME, 1, mqtt_sub_request_cb,
				arg);
		if (err != ERR_OK) {
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
		} else
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	} else {
		osDelay(2000);
		connect_to_server(client);
	}
}

static void mqtt_sub_request_cb(void *arg, err_t result) {
	switch (result) {
	case ERR_INPROGRESS:

		break;
	case ERR_OK:
	default:
		break;
	}
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic,
		uint32_t top_len) {

}

static void mqtt_incoming_data_cb(void *arg, const uint8_t *data, uint16_t len,
		uint8_t flags) {
	if (len > 0) {
		switch (data[0]) {
		case ACTION_RUN:
			deviceState.action = ACTION_RUN;
			break;
		case ACTION_STOP:
			deviceState.action = ACTION_STOP;
			break;
		default:
			break;
		}
	}
}

static void mqtt_pub_request_cb(void *arg, err_t result) {
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

////////////////display section/////////////////////////

void screenInitTask(void *args) {
	uint8_t status = screen_init();

	if (!status) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		osThreadExit();
	}

	screenRefreshCallback(NULL);
	//refresh the screen each 1 minute
	osTimerStart(screenRefreshTimHandle, 60000);
	osThreadExit();
}

//init eink-display and return stack for a display RAM
uint8_t screen_init(void) {
	if (screenBusyMutexHandle != NULL) {
		osStatus_t res = osMutexAcquire(screenBusyMutexHandle, osWaitForever);
		if (res == osOK) {
			DEV_Module_Init();
			EPD_2IN9BC_Init();
			EPD_2IN9BC_Clear();

			UWORD image_size = (
					(EPD_2IN9BC_WIDTH % 8 == 0) ?
							(EPD_2IN9BC_WIDTH / 8) : (EPD_2IN9BC_WIDTH / 8 + 1))
					* EPD_2IN9BC_HEIGHT;
			if ((BlackImage = (UBYTE*) pvPortMalloc(image_size)) == NULL) {
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
				EPD_2IN9BC_Sleep();
				DEV_Module_Exit();
				return 0;
			}
			if ((YellowImage = (UBYTE*) pvPortMalloc(image_size)) == NULL) {
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
				EPD_2IN9BC_Sleep();
				DEV_Module_Exit();
				return 0;
			}
			Paint_NewImage(BlackImage, EPD_2IN9BC_WIDTH, EPD_2IN9BC_HEIGHT, 90,
			BLACK);
			Paint_NewImage(YellowImage, EPD_2IN9BC_WIDTH, EPD_2IN9BC_HEIGHT, 90,
			BLACK);
			Paint_SelectImage(BlackImage);
			Paint_Clear(BLACK);
			Paint_SelectImage(YellowImage);
			Paint_Clear(BLACK);
			EPD_2IN9BC_Display(BlackImage, YellowImage);
		}
		osMutexRelease(screenBusyMutexHandle);
	} else
		return 0;
//	EPD_2IN9BC_Sleep();
	return 1;
}

///////////////utility//////////////////////////////

//return 96-bit unique ID of MCU in a char array
static void getDeviceID(char *name) {
	uint32_t id[3] = { 0 };
	id[0] = HAL_GetUIDw0();
	id[1] = HAL_GetUIDw1();
	id[2] = HAL_GetUIDw2();
	for (uint8_t i = 0, j = 0; i < 3; i++, j += 4) {
		char temp[4] = { 0 };
		itoa(id[i], temp, 10);
		for (uint8_t z = 0; z < 4; z++) {
			name[j + z] = temp[z];
		}
	}
}

static void makePubTopicName(char *out, size_t out_len) {
	if (out != NULL) {
		strncpy(out, PUB_TOPIC_NAME_PRED, sizeof(PUB_TOPIC_NAME_PRED));
		strncat(out, device.deviceID, sizeof(device.deviceID));
		strncat(out, PUB_TOPIC_NAME_POST_DATA,
				sizeof(PUB_TOPIC_NAME_POST_DATA));
	}
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc) {
//	osSemaphoreRelease(DMA2BusySemHandle);
//}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
