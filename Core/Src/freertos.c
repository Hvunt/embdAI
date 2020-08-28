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

#include "device.h"
#include "iwdg.h"
#include "rng.h"
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
//#define PUB_TOPIC_NAME "device/sensor"		//TODO replace "device" with MCU ID
//#define SUB_TOPIC_NAME "device/settings"	//TODO replace "device" with MCU ID
//#define PUB_TOPIC_NAME 	"/sensor"
#define SUB_TOPIC_NAME 	"devices/settings"

static const char PUB_TOPIC_NAME[] = "/sensor";

#define DEVICE_LOGIN	"test_device"
#define DEVICE_PASSWORD "4556"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//pool of memory for the eink display
UBYTE *BlackImage, *YellowImage;

extern RNG_HandleTypeDef hrng;
extern UART_HandleTypeDef huart3;
extern IWDG_HandleTypeDef hiwdg;

extern struct netif gnetif;

DeviceSettings_t device;
//osMutexId_t connectionBusy_Mutex;
//osMutexId_t displayBusy_Mutex;
//
//osSemaphoreId_t dataFactory_Sem;
//
//const osThreadAttr_t drawDataTask_attributes = {
//		.stack_size = 128 * 4
//};
/* USER CODE END Variables */
/* Definitions for initTaskName */
osThreadId_t initTaskNameHandle;
const osThreadAttr_t initTaskName_attributes = { .name = "initTaskName",
		.priority = (osPriority_t) osPriorityNormal, .stack_size = 128 * 4 };
/* Definitions for dataSend */
osThreadId_t dataSendHandle;
const osThreadAttr_t dataSend_attributes = { .name = "dataSend", .priority =
		(osPriority_t) osPriorityLow4, .stack_size = 512 * 4 };
/* Definitions for collectData */
osThreadId_t collectDataHandle;
const osThreadAttr_t collectData_attributes = { .name = "collectData",
		.priority = (osPriority_t) osPriorityLow, .stack_size = 256 * 4 };
/* Definitions for sensorsData */
osMessageQueueId_t sensorsDataHandle;
uint8_t sensorsDataBuffer[1000 * sizeof(uint8_t)];
osStaticMessageQDef_t sensorsDataControlBlock;
const osMessageQueueAttr_t sensorsData_attributes = { .name = "sensorsData",
		.cb_mem = &sensorsDataControlBlock, .cb_size =
				sizeof(sensorsDataControlBlock), .mq_mem = &sensorsDataBuffer,
		.mq_size = sizeof(sensorsDataBuffer) };
/* Definitions for dataBusyMutex */
osMutexId_t dataBusyMutexHandle;
const osMutexAttr_t dataBusyMutex_attributes = { .name = "dataBusyMutex" };
/* Definitions for screenBusyMutex */
osMutexId_t screenBusyMutexHandle;
const osMutexAttr_t screenBusyMutex_attributes = { .name = "screenBusyMutex" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void screenInitTask(void *args);
void watchdogTask(void *args);
//void sendDataTask(void *arg);
//void writeDataToSDTask(void *arg);
//void collectDataTask(void *arg);
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

//eink-display function prototype
uint8_t screen_init();

/* USER CODE END FunctionPrototypes */

void initTask(void *argument);
void dataSendTask(void *argument);
void collectDataTask(void *argument);

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

	/* USER CODE BEGIN RTOS_MUTEX */
//	connectionBusy_Mutex = osMutexNew(NULL);
//	displayBusy_Mutex = osMutexNew(NULL);
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
//	dataFactory_Sem = osSemaphoreNew(3, 0, NULL);
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of sensorsData */
	sensorsDataHandle = osMessageQueueNew(1000, sizeof(uint8_t),
			&sensorsData_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of initTaskName */
	initTaskNameHandle = osThreadNew(initTask, NULL, &initTaskName_attributes);

	/* creation of dataSend */
	dataSendHandle = osThreadNew(dataSendTask, NULL, &dataSend_attributes);

	/* creation of collectData */
	collectDataHandle = osThreadNew(collectDataTask, NULL,
			&collectData_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	const osThreadAttr_t screenInit_attributes = { .name = "screenInit",
			.priority = (osPriority_t) osPriorityLow2, .stack_size = 256
					* 4 };
//	const osThreadAttr_t watchdogTask_attributes = { .name = "watchdogTask",
//			.priority = (osPriority_t) osPriorityNormal, .stack_size = 64 * 4 };
//	const osThreadAttr_t sendData_attributes = { .name = "sendData", .priority =
//			(osPriority_t) osPriorityNormal, .stack_size = 256 * 4 };
//	const osThreadAttr_t writeDataToSD_attributes = { .name = "writeDataToSD",
//			.priority = (osPriority_t) osPriorityNormal, .stack_size = 256 * 4 };
//	const osThreadAttr_t collectData_attributes = { .name = "collectData",
//			.priority = (osPriority_t) osPriorityNormal, .stack_size = 256 * 4 };
//
	osThreadNew(screenInitTask, NULL, &screenInit_attributes);
//	osThreadNew(watchdogTask, NULL, &watchdogTask_attributes);
//	osThreadNew(sendDataTask, NULL, &sendData_attributes);
//	osThreadNew(writeDataToSDTask, NULL, &writeDataToSD_attributes);
//	osThreadNew(collectDataTask, NULL, &collectData_attributes);

	/* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_initTask */
/**
 * @brief  Initialization of software part of the device.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_initTask */
void initTask(void *argument) {
	/* USER CODE BEGIN initTask */
//	char deviceID[12] = { 0 };
//	getDeviceID(deviceID);
//	deviceSettingsInit(&device);
//	deviceSetID(&device, deviceID);
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
void dataSendTask(void *argument) {
	/* USER CODE BEGIN dataSendTask */
	MX_LWIP_Init();
	osDelay(3000);
	mqtt_client_t *client = mqtt_client_new();
	if (client != NULL) {
		connect_to_server(client);
	}
	osDelay(1000);
	/* Infinite loop */
	char pub_top_name[sizeof(device.deviceID) - 1 + sizeof(PUB_TOPIC_NAME)];
	strncpy(pub_top_name, device.deviceID, sizeof(device.deviceID));
	for (uint8_t i = sizeof(device.deviceID) - 1, j = 0; i < sizeof(pub_top_name);
			++i, ++j)
		pub_top_name[i] = PUB_TOPIC_NAME[j];

	char temp[10] = { 0 };
	uint16_t errors = 0;
	for (;;) {
		if(mqtt_client_is_connected(client)){
			if (dataBusyMutexHandle != NULL) {
				osStatus_t result = osMutexAcquire(dataBusyMutexHandle, 20);
				if (result == osOK) {
					err_t err = mqtt_publish(client, pub_top_name,
							(uint8_t*) sensorsDataBuffer, 200, 2, 0,
							mqtt_pub_request_cb, NULL);
					if (err != ERR_OK) {
						HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
						errors++;

						snprintf(temp, sizeof(temp), "%d\n", errors);
						HAL_UART_Transmit(&huart3, (uint8_t*) temp,
								sizeof(temp), 0x10);
					} else {
						HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,
								GPIO_PIN_RESET);
					}
				}
				osMutexRelease(dataBusyMutexHandle);
			}
		} else {
//			uint8_t state = client->conn_state;
//			if (state == 0){
				osDelay(2000);
				connect_to_server(client);
//			}
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
void collectDataTask(void *argument) {
	/* USER CODE BEGIN collectDataTask */
	/* Infinite loop */
//	uint8_t state = HAL_RNG_GetState(&hrng);
	MX_RNG_Init();
	for (;;) {
		if (dataBusyMutexHandle != NULL) {
			osStatus_t res = osMutexAcquire(dataBusyMutexHandle, 20);
			if (res == osOK) {
				for (uint16_t i = 0; i < sensorsData_attributes.mq_size; i++) {
//					sensorsDataBuffer[i] = (uint8_t) HAL_RNG_GetRandomNumber(
//							&hrng);
					HAL_RNG_GenerateRandomNumber(&hrng, (uint32_t *) &sensorsDataBuffer[i]);
				}

			}
			osMutexRelease(dataBusyMutexHandle);
		}
		osDelay(100);
	}
	/* USER CODE END collectDataTask */
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
//void sendDataTask(void *arg) {
//	mqtt_client_t *client = mqtt_client_new();
//	if(client != NULL){
//		connect_to_server(client);
//	}
//	osDelay(100);
//	char deviceID[12] = {0};
//	getDeviceID(deviceID);
//	osThreadNew(drawDataTask, (void *) deviceID, &drawDataTask_attributes);
//	for (;;) {
//
//	}
//}
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
	HAL_UART_Transmit(&huart3, (uint8_t*) data, len, 0x100);
}

static void mqtt_pub_request_cb(void *arg, err_t result) {
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

////////////////display section/////////////////////////

//draw data on the eink-display
void drawMessageTask(void *args) {
	einkDrawData_t *data = (einkDrawData_t*) args;
	if (screenBusyMutexHandle != NULL) {
		osStatus_t res = osMutexAcquire(screenBusyMutexHandle, osWaitForever);
		if (res == osOK) {
			char id[13] = {0};
			switch (data->type) {
			case EDDT_CONNECTION_STATUS:

				break;
			case EDDT_ERROR:

				break;
			case EDDT_IP:

				break;
			case EDDT_STATUS:

				break;
			case EDDT_TIME:

				break;
			case EDDT_UUID:
				deviceGetID(&device, id, sizeof(id));
				Paint_DrawString_EN(5, 20, id, &Font16, BLACK,
				WHITE);
				break;
			}
			Paint_SelectImage(BlackImage);
			EPD_2IN9BC_Display(BlackImage, YellowImage);
//			DEV_Delay_ms(2000);
//			EPD_2IN9BC_Sleep();
			osMutexRelease(screenBusyMutexHandle);
		}

	}

	osThreadExit();
}

void screenInitTask(void *args) {
	osStatus_t res = osMutexAcquire(screenBusyMutexHandle, osWaitForever);
	if (res == osOK) {
		uint8_t status = screen_init();
		einkDrawData_t data;
		data.type = EDDT_UUID;
		osThreadNew(drawMessageTask, &data, NULL);
		if (!status) {
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			osThreadExit();
		}
		osMutexRelease(screenBusyMutexHandle);
	}
	osThreadExit();
}

//init eink-display and return stack for a display RAM
uint8_t screen_init(void) {
	DEV_Module_Init();
	EPD_2IN9BC_Init();
	EPD_2IN9BC_Clear();
//	DEV_Delay_ms(2000);

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
	Paint_NewImage(BlackImage, EPD_2IN9BC_WIDTH, EPD_2IN9BC_HEIGHT, 270, BLACK);
	Paint_NewImage(YellowImage, EPD_2IN9BC_WIDTH, EPD_2IN9BC_HEIGHT, 270,
	BLACK);
	Paint_SelectImage(BlackImage);
	Paint_Clear(BLACK);
	Paint_SelectImage(YellowImage);
	Paint_Clear(BLACK);
	EPD_2IN9BC_Display(BlackImage, YellowImage);
//	DEV_Delay_ms(2000);
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
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
