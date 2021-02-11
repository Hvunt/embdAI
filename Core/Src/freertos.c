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

#include "lwip.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "lwip/netif.h"
#include "lwip/apps/sntp.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"

#include "GUI_Paint.h"
#include "EPD_2in9bc.h"
#include "MAX31865.h"
#include "device.h"

#include "device.h"
//#include "iwdg.h"
#include "rng.h"
#include "tim.h"
#include "pepega.h"
//#include "fatfs.h"

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
//static const char PUB_TOPIC_NAME_POST_SETTINGS[] = "/settings";
static const char PUB_TOPIC_NAME_PRED[] = "devices/";

#define DEVICE_LOGIN	"test_device"
#define DEVICE_PASSWORD "4556"

#define FRAME_SIZE 				300
#define FRAME_HEAD_LENGTH 		6

#define MSG_DATA_READY 				0x01
#define MSG_COLLECT 				0x02
#define MSG_PACKET_HAS_BEEN_SENT 	0x03
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
//extern IWDG_HandleTypeDef hiwdg;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim6;

extern struct netif gnetif;

DeviceSettings_t device;
DeviceAction_t deviceState;

Max31865_t max_1, max_2;
volatile uint16_t HATS_temp_1 = 0, HATS_temp_2 = 0;
volatile uint8_t HATS_new_data = 0;

volatile uint32_t measurements_counter = 0;

const osThreadAttr_t screenInit_attributes = { .name = "screenInit", .priority =
		(osPriority_t) osPriorityNormal, .stack_size = 256 * 4 };

osEventFlagsId_t data_evnt_id;
/* USER CODE END Variables */
/* Definitions for init */
osThreadId_t initHandle;
const osThreadAttr_t init_attributes = { .name = "init", .priority =
		(osPriority_t) osPriorityLow7, .stack_size = 128 * 4 };
/* Definitions for dataSend */
osThreadId_t dataSendHandle;
const osThreadAttr_t dataSend_attributes = { .name = "dataSend", .priority =
		(osPriority_t) osPriorityBelowNormal, .stack_size = 650 * 4 };
/* Definitions for collectData */
osThreadId_t collectDataHandle;
const osThreadAttr_t collectData_attributes =
		{ .name = "collectData", .priority =
				(osPriority_t) osPriorityBelowNormal, .stack_size = 128 * 4 };
/* Definitions for logData */
osThreadId_t logDataHandle;
const osThreadAttr_t logData_attributes = { .name = "logData", .priority =
		(osPriority_t) osPriorityLow4, .stack_size = 128 * 4 };
/* Definitions for logStatus */
osThreadId_t logStatusHandle;
const osThreadAttr_t logStatus_attributes = { .name = "logStatus", .priority =
		(osPriority_t) osPriorityLow4, .stack_size = 128 * 4 };
/* Definitions for screenRefresh */
osThreadId_t screenRefreshHandle;
const osThreadAttr_t screenRefresh_attributes = { .name = "screenRefresh",
		.priority = (osPriority_t) osPriorityLow6, .stack_size = 200 * 4 };
/* Definitions for HATSCollect */
osThreadId_t HATSCollectHandle;
const osThreadAttr_t HATSCollect_attributes = { .name = "HATSCollect",
		.priority = (osPriority_t) osPriorityLow6, .stack_size = 150 * 4 };
/* Definitions for sensorsData */
osMessageQueueId_t sensorsDataHandle;
uint8_t sensorsDataBuffer[27000 * sizeof(uint8_t)];
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
/* Definitions for sdBusyMutex */
osMutexId_t sdBusyMutexHandle;
const osMutexAttr_t sdBusyMutex_attributes = { .name = "sdBusyMutex" };
/* Definitions for HATSMutex */
osMutexId_t HATSMutexHandle;
const osMutexAttr_t HATSMutex_attributes = { .name = "HATSMutex" };
/* Definitions for DMA2BusySem */
osSemaphoreId_t DMA2BusySemHandle;
const osSemaphoreAttr_t DMA2BusySem_attributes = { .name = "DMA2BusySem" };
/* Definitions for dataProcessingSem */
osSemaphoreId_t dataProcessingSemHandle;
const osSemaphoreAttr_t dataProcessingSem_attributes = { .name =
		"dataProcessingSem" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void screenInitTask(void *args);
void watchdogTask(void *args);
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
void load_bootloader();

//eink-display functions prototype
uint8_t screen_init();

/* USER CODE END FunctionPrototypes */

void initTask(void *argument);
void dataSendTask(void *argument);
void collectDataTask(void *argument);
void logDataTask(void *argument);
void logStatusTask(void *argument);
void screenRefreshTask(void *argument);
void HATSCollectTask(void *argument);

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

	data_evnt_id = osEventFlagsNew(NULL);
	/* USER CODE END Init */
	/* Create the mutex(es) */
	/* creation of dataBusyMutex */
	dataBusyMutexHandle = osMutexNew(&dataBusyMutex_attributes);

	/* creation of screenBusyMutex */
	screenBusyMutexHandle = osMutexNew(&screenBusyMutex_attributes);

	/* creation of sdBusyMutex */
	sdBusyMutexHandle = osMutexNew(&sdBusyMutex_attributes);

	/* creation of HATSMutex */
	HATSMutexHandle = osMutexNew(&HATSMutex_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of DMA2BusySem */
	DMA2BusySemHandle = osSemaphoreNew(1, 1, &DMA2BusySem_attributes);

	/* creation of dataProcessingSem */
	dataProcessingSemHandle = osSemaphoreNew(2, 2,
			&dataProcessingSem_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of sensorsData */
	sensorsDataHandle = osMessageQueueNew(27000, sizeof(uint8_t),
			&sensorsData_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of init */
	initHandle = osThreadNew(initTask, NULL, &init_attributes);

	/* creation of dataSend */
	dataSendHandle = osThreadNew(dataSendTask, NULL, &dataSend_attributes);

	/* creation of collectData */
	collectDataHandle = osThreadNew(collectDataTask, NULL,
			&collectData_attributes);

	/* creation of logData */
	logDataHandle = osThreadNew(logDataTask, NULL, &logData_attributes);

	/* creation of logStatus */
	logStatusHandle = osThreadNew(logStatusTask, NULL, &logStatus_attributes);

	/* creation of screenRefresh */
	screenRefreshHandle = osThreadNew(screenRefreshTask, NULL,
			&screenRefresh_attributes);

	/* creation of HATSCollect */
	HATSCollectHandle = osThreadNew(HATSCollectTask, NULL,
			&HATSCollect_attributes);

	/* USER CODE BEGIN RTOS_THREADS */

	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

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
//	f_mount(&SDFatFS, "", 0);
//	ff_del_syncobj(&SDFatFS.sobj);
//	SDFatFS.sobj = DMA2BusySemHandle;
//	for (;;) {
//		osDelay(1000);
//	}
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

	enum {
		eTCP_DISCONNECTED, eTCP_CONNECTING, eMQTT_CONNECTING, eMQTT_CONNECTED
	};

	//make topic name: device/[UUID]/sensor
	size_t pub_top_name_len = sizeof(device.deviceID) - 2
			+ sizeof(PUB_TOPIC_NAME_PRED) + sizeof(PUB_TOPIC_NAME_POST_DATA);
	char pub_top_name[pub_top_name_len];
	makePubTopicName(pub_top_name, pub_top_name_len);

	uint16_t sending_errors = 0;
	uint8_t tx_buffer[FRAME_SIZE + FRAME_HEAD_LENGTH] = { 0 };
	uint8_t packets_count = sensorsData_attributes.mq_size / FRAME_SIZE;

	device.device_status = DEVICE_STATUS_AWAITING;
//	deviceState.action = ACTION_RUN;
	for (;;) {
//		if (mqtt_client_is_connected(client)) {
		if (client->conn_state == eMQTT_CONNECTED) {
			HAL_GPIO_WritePin(LD2_GPIO_Port,
			LD2_Pin, GPIO_PIN_RESET);
			if (deviceState.action == ACTION_RUN) {
				if (dataBusyMutexHandle != NULL) {
					osEventFlagsSet(data_evnt_id, MSG_COLLECT);
					uint32_t event_flag = osEventFlagsWait(data_evnt_id,
					MSG_DATA_READY, osFlagsWaitAny, osWaitForever);
					if (event_flag == MSG_DATA_READY) {
						osStatus_t result = osMutexAcquire(dataBusyMutexHandle,
						osWaitForever);
						if (result == osOK) {
//							packets_count = sensorsData_attributes.mq_size / FRAME_SIZE;
							for (uint32_t i = 0, frame_count = 1;
									frame_count <= packets_count;
									frame_count++, i +=
									FRAME_SIZE) {
								tx_buffer[0] = device.measurements_counter
										<< 24;
								tx_buffer[1] = device.measurements_counter
										<< 16;
								tx_buffer[2] = device.measurements_counter << 8;
								tx_buffer[3] = device.measurements_counter;
								tx_buffer[4] = frame_count;
								tx_buffer[5] = packets_count;
								for (uint32_t j = 0;
										j < FRAME_SIZE + FRAME_HEAD_LENGTH;
										j++) {
									tx_buffer[j + FRAME_HEAD_LENGTH] =
											sensorsDataBuffer[i + j];
								}
								err_t err = ERR_CONN;
								while (err != ERR_OK) {
									err = mqtt_publish(client, pub_top_name,
											(uint8_t*) tx_buffer,
											sizeof(tx_buffer), 0, 0,
											mqtt_pub_request_cb, NULL);

									if (err != ERR_OK) {
										HAL_GPIO_WritePin(LD1_GPIO_Port,
										LD1_Pin, GPIO_PIN_SET);
										sending_errors++;
									} else {
										HAL_GPIO_WritePin(LD1_GPIO_Port,
										LD1_Pin, GPIO_PIN_RESET);
									}
									osDelay(5);
								}
								event_flag = osEventFlagsWait(data_evnt_id,
								MSG_PACKET_HAS_BEEN_SENT, osFlagsWaitAny,
								osWaitForever);
								if (event_flag != MSG_PACKET_HAS_BEEN_SENT){
//									while(1){
										HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
										osDelay(100);
//									}
								}
							}
							osMutexRelease(dataBusyMutexHandle);
							device.measurements_counter++;
						}
					}
				} else {
					HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
				}

				osDelay(device.time_interval);
			} else
				osDelay(10);
		} else if (client->conn_state == eTCP_DISCONNECTED) {
			for (uint8_t i = 0; i < 4; i++) {
				HAL_GPIO_TogglePin(LD2_GPIO_Port,
				LD2_Pin);
				osDelay(500);
			}
			connect_to_server(client);
		/*} else if (client->conn_state == eMQTT_CONNECTING){
			osDelay(500);*/
		} else {
			HAL_GPIO_WritePin(LD2_GPIO_Port,
			LD2_Pin, GPIO_PIN_SET);
		}
		if (deviceState.action == ACTION_UPDATE) {
			load_bootloader();
		}
//		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
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
	uint16_t temp = 0xFF;
	for (;;) {

		if (dataBusyMutexHandle != NULL) {
			if (deviceState.action == ACTION_RUN) {
				uint32_t event_flag = osEventFlagsWait(data_evnt_id,
				MSG_COLLECT, osFlagsWaitAny, osWaitForever);
				if (event_flag == MSG_COLLECT) {
					osStatus_t res = osMutexAcquire(dataBusyMutexHandle,
					osWaitForever);
					if (res == osOK) {
						clean_buff(sensorsDataBuffer,
								sensorsData_attributes.mq_size);
//						for (uint32_t i = 0; i < sensorsData_attributes.mq_size; i++)
//							sensorsDataBuffer[i] = 0;

//						uint32_t start_ticks = osKernelGetTickCount();
//						uint32_t now_ticks = start_ticks;

						for (uint16_t i = 0;
								(i < sensorsData_attributes.mq_size);) {
							// ///collect data from ADC/// //
							uint16_t data[7] = { 0 };
							osSemaphoreAcquire(DMA2BusySemHandle,
							osWaitForever);
							HAL_ADC_Start_DMA(&hadc3, (uint32_t*) data, 7);
							osSemaphoreAcquire(DMA2BusySemHandle,
							osWaitForever);
							osSemaphoreRelease(DMA2BusySemHandle);
							for (uint8_t j = 0; j < 7; j++, i += 2) {
								sensorsDataBuffer[i] = data[j] >> 8;
								sensorsDataBuffer[i + 1] = data[j];
							}

							// ///collect data from MAX31865/// //
							osMutexAcquire(HATSMutexHandle, osWaitForever);
							if (HATS_temp_1 < 1000) {
								temp = HATS_temp_1;
								sensorsDataBuffer[i] = temp >> 8;
								sensorsDataBuffer[i + 1] = temp;
							} else {
								sensorsDataBuffer[i] = 0xFF;
								sensorsDataBuffer[i + 1] = 0xFF;
							}

							i += 2;
							if (HATS_temp_2 < 1000) {
								temp = HATS_temp_2;
								sensorsDataBuffer[i] = temp >> 8;
								sensorsDataBuffer[i + 1] = temp;
							} else {
								sensorsDataBuffer[i] = 0xFF;
								sensorsDataBuffer[i + 1] = 0xFF;
							}
							osMutexRelease(HATSMutexHandle);
							i += 2;
						}

//					now_ticks = osKernelGetTickCount();
//					now_ticks -= start_ticks;

						osMutexRelease(dataBusyMutexHandle);
						osEventFlagsSet(data_evnt_id, MSG_DATA_READY);
					}
				}
			} else
				osDelay(1);
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
void logDataTask(void *argument) {
	/* USER CODE BEGIN logDataTask */
	osThreadExit();
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
void logStatusTask(void *argument) {
	/* USER CODE BEGIN logStatusTask */
	osThreadExit();
//	char log_info[255];
//	FIL logFile;
//	FRESULT f_res;
//	UINT bytes_written;
//	if(f_mount(&SDFatFS, "", 0) != FR_OK){
//		while(1){
//			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//			osDelay(100);
//		}
//	}
	for (;;) {
//		if (sdBusyMutexHandle != NULL) {
//			osStatus_t res = osSemaphoreAcquire(sdBusyMutexHandle, osWaitForever);
//			if (res == osOK) {
//				char id[13] = { 0 };
//				deviceGetID(&device, id, sizeof(id));
//				sprintf(log_info, "ID: %s\n", id);
//				if (f_open(&logFile, "LOG.LOG", FA_OPEN_APPEND | FA_WRITE)
//						== FR_OK) {
//					f_sync(&logFile);
//					f_res = f_write(&logFile, log_info, strlen(log_info),
//							&bytes_written);
//					if (f_res != FR_OK) {
//						HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
//					}
//					f_sync(&logFile);
//					f_close(&logFile);
//				}
//				osSemaphoreRelease(sdBusyMutexHandle);
//			}
//		}
		osDelay(1000);
	}
	/* USER CODE END logStatusTask */
}

/* USER CODE BEGIN Header_screenRefreshTask */
/**
 * @brief Function implementing the screenRefresh thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_screenRefreshTask */
void screenRefreshTask(void *argument) {
	/* USER CODE BEGIN screenRefreshTask */
	uint8_t status = screen_init();

	if (!status) {
		while (1) {
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			osDelay(1000);
		}
	}

	for (;;) {
		if (screenBusyMutexHandle != NULL) {
			osStatus_t res = osMutexAcquire(screenBusyMutexHandle,
			osWaitForever);
			if (res == osOK) {
				Paint_SelectImage(YellowImage);

//				Paint_DrawBitMap(gImage_pepega);
				char data[40] = { 0 };

//				Paint_DrawLine(1, 1, 100, 0, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
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
				sprintf(data, "My IP: ");
				strcat(data, ip4addr_ntoa((const ip4_addr_t*) &gnetif.ip_addr));
				Paint_DrawString_EN(5, 52, data, &Font16, BLACK,
				WHITE);

				EPD_2IN9BC_Display(BlackImage, YellowImage);
				osMutexRelease(screenBusyMutexHandle);
			}
		}
		osDelay(30000);
	}
	/* USER CODE END screenRefreshTask */
}

/* USER CODE BEGIN Header_HATSCollectTask */
/**
 * @brief Function implementing the HATSCollect thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_HATSCollectTask */
void HATSCollectTask(void *argument) {
	/* USER CODE BEGIN HATSCollectTask */
	Max31865_init(&max_1, &hspi4, SPI_TCS1_GPIO_Port, SPI_TCS1_Pin, 2, 50);
	Max31865_init(&max_2, &hspi4, SPI_TCS2_GPIO_Port, SPI_TCS2_Pin, 2, 50);

	float temp = 0, temp2 = 0;
	osStatus_t res;
	for (;;) {
		Max31865_readTempC(&max_1, &temp);
		Max31865_readTempC(&max_2, &temp2);
		res = osMutexAcquire(HATSMutexHandle, osWaitForever);
		if (res == osOK) {
			HATS_temp_1 = (float) temp * 10;
			HATS_temp_2 = (float) temp2 * 10;
			HATS_new_data = 1;
			osMutexRelease(HATSMutexHandle);
		}
		osDelay(500);
	}
	/* USER CODE END HATSCollectTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void watchdogTask(void *args) {
//	MX_IWDG_Init();
	osThreadExit();
	for (;;) {
		osDelay(490);
//		HAL_IWDG_Refresh(&hiwdg);
	}
}
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
	ci.keep_alive = 30;
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
			device.device_status = DEVICE_STATUS_COLLECT_DATA;
			break;
		case ACTION_STOP:
			deviceState.action = ACTION_STOP;
			device.device_status = DEVICE_STATUS_AWAITING;
			break;
		case ACTION_UPDATE:
			deviceState.action = ACTION_UPDATE;

		default:
			break;
		}
	}
}

static void mqtt_pub_request_cb(void *arg, err_t result) {
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	osEventFlagsSet(data_evnt_id, MSG_PACKET_HAS_BEEN_SENT);
}

////////////////display section/////////////////////////

//void screenInitTask(void *args) {
//	uint8_t status = screen_init();
//
//	if (!status) {
//		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//		osThreadExit();
//	}
//
////	screenRefreshCallback(NULL);
////refresh the screen each 1 minute
////	if (screenRefreshTimHandle != NULL)
////		osTimerStart(screenRefreshTimHandle, 30000);
////	osDelay(60000);
//	osThreadExit();
//}

//init eink-display and return stack for a display RAM
uint8_t screen_init(void) {
	if (screenBusyMutexHandle != NULL) {
		osStatus_t res = osMutexAcquire(screenBusyMutexHandle,
		osWaitForever);
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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc) {
	osSemaphoreRelease(DMA2BusySemHandle);
}

//////////////load bootloader///////////////////////
void load_bootloader() {
	FLASH_EraseInitTypeDef FLASH_EraseInitStruct;
	uint32_t sectornb = 0;

	FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	FLASH_EraseInitStruct.Sector = FLASH_SECTOR_11;
	FLASH_EraseInitStruct.NbSectors = 1;
	FLASH_EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASH_Unlock();
	if (HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &sectornb) != HAL_OK)
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
			(uint32_t) LOAD_BOOTLOADER_ADDRESS, 1);
	HAL_FLASH_Lock();
	NVIC_SystemReset();
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
