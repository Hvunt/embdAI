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
//#include "lwip/apps/sntp.h"
#include "lwip/sockets.h"
#include "lwip/tcpbase.h"

#include "GUI_Paint.h"
#include "EPD_2in9bc.h"
#include "MAX31865.h"
#include "device.h"

#include "device.h"
//#include "iwdg.h"
#include "tim.h"
//#include "pepega.h"
//#include "fatfs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
enum einkDrawDataTypes {
	EDDT_IP = 0, EDDT_CONNECTION_STATUS, EDDT_ERROR, EDDT_STATUS, EDDT_TIME, EDDT_UUID
};
typedef struct einkDrawData {
	uint8_t type;
	uint8_t *data;
	uint16_t length;
} einkDrawData_t;

typedef struct socketClient {
	uint32_t soc;
	uint32_t port;
} socketClient_t;

typedef struct dataPacket {
	uint32_t data_length;
	uint8_t action;
	uint8_t subaction;
} dataPacket_t, *pdataPacket_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILESERVER_ADDRESS		"192.168.7.25"
#define SERVER_PORT				2412

#define MSG_DATA_READY 				(1<<0)
#define MSG_COLLECT					(1<<1)
#define MSG_PACKET_HAS_BEEN_SENT	(1<<2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define clean_buff(buf, len) for (uint32_t i = 0; i < len;i++) buf[i] = 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//pool of memory for the eink display
UBYTE *BlackImage, *YellowImage;

//extern IWDG_HandleTypeDef hiwdg;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;

extern struct netif gnetif;

DeviceSettings_t device;
DeviceAction_t deviceState;

Max31865_t max_1, max_2;
//volatile uint16_t HATS_temp_1 = 0xFFFF, HATS_temp_2 = 0xFFFF;
volatile uint16_t HATS_temp[2] = { 0xFFFF };
volatile uint8_t HATS_new_data = 0;

volatile uint32_t measurements_counter = 0;

const osThreadAttr_t screenInit_attributes = { .name = "screenInit", .priority = (osPriority_t) osPriorityLow4, .stack_size = 256 * 4 };
osThreadId_t receiveDataHandle;
const osThreadAttr_t receiveData_attributes = { .name = "receiveSocket", .priority = (osPriority_t) osPriorityBelowNormal, .stack_size = 256
		* 4 };

osEventFlagsId_t data_evnt_id;

volatile socketClient_t *client = NULL;

/////////////DEBUG/////////////////
#define DATA_DISPL_READY_FLAG (1<<1)
#define DATA_ACCEL_READY_FLAG (1<<2)

#define DATA_DISPL_HALF_READY_FLAG (1<<3)
#define DATA_ACCEL_HALF_READY_FLAG (1<<4)

uint16_t data_displ[2000 * 2] = { 0 };
uint16_t data_accel[20000 * 3] = { 0 };
uint16_t data_temp[2 * 2] = { 0 };

volatile uint32_t tim6_time = 0;
/////////////DEBUG/////////////////

/* USER CODE END Variables */
/* Definitions for init */
osThreadId_t initHandle;
const osThreadAttr_t init_attributes = { .name = "init", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for dataSend */
osThreadId_t dataSendHandle;
const osThreadAttr_t dataSend_attributes = { .name = "dataSend", .stack_size = 650 * 4, .priority = (osPriority_t) osPriorityBelowNormal, };
/* Definitions for collectData */
osThreadId_t collectDataHandle;
const osThreadAttr_t collectData_attributes = { .name = "collectData", .stack_size = 128 * 4, .priority =
		(osPriority_t) osPriorityBelowNormal, };
/* Definitions for logData */
osThreadId_t logDataHandle;
const osThreadAttr_t logData_attributes = { .name = "logData", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow4, };
/* Definitions for logStatus */
osThreadId_t logStatusHandle;
const osThreadAttr_t logStatus_attributes = { .name = "logStatus", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow4, };
/* Definitions for screenRefresh */
osThreadId_t screenRefreshHandle;
const osThreadAttr_t screenRefresh_attributes =
		{ .name = "screenRefresh", .stack_size = 200 * 4, .priority = (osPriority_t) osPriorityLow6, };
/* Definitions for HATSCollect */
osThreadId_t HATSCollectHandle;
const osThreadAttr_t HATSCollect_attributes = { .name = "HATSCollect", .stack_size = 150 * 4, .priority = (osPriority_t) osPriorityLow6, };
/* Definitions for sensorsData */
osMessageQueueId_t sensorsDataHandle;
uint8_t sensorsDataBuffer[128004 * sizeof(uint8_t)];
osStaticMessageQDef_t sensorsDataControlBlock;
const osMessageQueueAttr_t sensorsData_attributes = { .name = "sensorsData", .cb_mem = &sensorsDataControlBlock, .cb_size =
		sizeof(sensorsDataControlBlock), .mq_mem = &sensorsDataBuffer, .mq_size = sizeof(sensorsDataBuffer) };
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
/* Definitions for socketMutex */
osMutexId_t socketMutexHandle;
const osMutexAttr_t socketMutex_attributes = { .name = "socketMutex" };
/* Definitions for DMA2BusySem */
osSemaphoreId_t DMA2BusySemHandle;
const osSemaphoreAttr_t DMA2BusySem_attributes = { .name = "DMA2BusySem" };
/* Definitions for dataProcessingSem */
osSemaphoreId_t dataProcessingSemHandle;
const osSemaphoreAttr_t dataProcessingSem_attributes = { .name = "dataProcessingSem" };
/* Definitions for dataReadyEvent */
osEventFlagsId_t dataReadyEventHandle;
const osEventFlagsAttr_t dataReadyEvent_attributes = { .name = "dataReadyEvent" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void screenInitTask(void *args);
void watchdogTask(void *args);
void drawMessageTask(void *message);

void connect_to_server(socketClient_t *client);
static void prepare_to_close_connection(void);
//void receiveDataTask(void *arg);
void decode_packet(char *in_data, uint32_t length);
//static void get_adc_data(ADC_HandleTypeDef *adc, uint16_t *out_buffer, uint32_t length);

//utility
static void getDeviceID(char *name);
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

	/* creation of socketMutex */
	socketMutexHandle = osMutexNew(&socketMutex_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of DMA2BusySem */
	DMA2BusySemHandle = osSemaphoreNew(1, 1, &DMA2BusySem_attributes);

	/* creation of dataProcessingSem */
	dataProcessingSemHandle = osSemaphoreNew(2, 2, &dataProcessingSem_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of sensorsData */
	sensorsDataHandle = osMessageQueueNew(128004, sizeof(uint8_t), &sensorsData_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of init */
	initHandle = osThreadNew(initTask, NULL, &init_attributes);

	/* creation of dataSend */
	dataSendHandle = osThreadNew(dataSendTask, NULL, &dataSend_attributes);

	/* creation of collectData */
	collectDataHandle = osThreadNew(collectDataTask, NULL, &collectData_attributes);

	/* creation of logData */
	logDataHandle = osThreadNew(logDataTask, NULL, &logData_attributes);

	/* creation of logStatus */
	logStatusHandle = osThreadNew(logStatusTask, NULL, &logStatus_attributes);

	/* creation of screenRefresh */
	screenRefreshHandle = osThreadNew(screenRefreshTask, NULL, &screenRefresh_attributes);

	/* creation of HATSCollect */
	HATSCollectHandle = osThreadNew(HATSCollectTask, NULL, &HATSCollect_attributes);

	/* USER CODE BEGIN RTOS_THREADS */

	/* USER CODE END RTOS_THREADS */

	/* creation of dataReadyEvent */
	dataReadyEventHandle = osEventFlagsNew(&dataReadyEvent_attributes);

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_initTask */
/**
 * @brief  Function implementing the init thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_initTask */
void initTask(void *argument) {
	/* USER CODE BEGIN initTask */
	osThreadExit();
//	for (;;) {
//		osDelay(1);
//	}
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

	while (gnetif.ip_addr.addr == 0) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		osDelay(500);
	}
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	client = (socketClient_t*) mem_calloc(1, sizeof(socketClient_t));
	connect_to_server((socketClient_t*) client);
	uint16_t sending_errors = 0;

	device.device_status = DEVICE_STATUS_AWAITING;
	ssize_t sended;
	for (;;) {
		if (deviceState.action == ACTION_GET_D) {
			if (dataBusyMutexHandle != NULL) {
				osEventFlagsSet(data_evnt_id, MSG_COLLECT);
				uint32_t event_flag = osEventFlagsWait(data_evnt_id, MSG_DATA_READY, osFlagsWaitAny, osWaitForever);
				if (event_flag == MSG_DATA_READY) {
					dataPacket_t packet;
//						packet.data = NULL;
					packet.action = ACTION_GET_D; // requsted action from server
					packet.subaction = 0;
					packet.data_length = sizeof(sensorsDataBuffer);
					sended = write(client->soc, &packet, sizeof(packet));
					osEventFlagsWait(data_evnt_id, MSG_PACKET_HAS_BEEN_SENT, osFlagsWaitAny, 10000);
					sended = write(client->soc, sensorsDataBuffer, packet.data_length);
					osEventFlagsWait(data_evnt_id, MSG_PACKET_HAS_BEEN_SENT, osFlagsWaitAny, 10000);
					if (sended <= 0) {
						HAL_GPIO_WritePin(LD1_GPIO_Port,
						LD1_Pin, GPIO_PIN_SET);
						sending_errors++;
						//TODO: Error handler
//							if (sending_errors > 10) {
//								shutdown(client->soc, SHUT_RDWR);
//								osDelay(500);
//								close(client->soc);
//								osDelay(500);
//								client->soc = -1;
//								connect_to_server((socketClient_t*) client);
//							}
					} else {
						HAL_GPIO_WritePin(LD1_GPIO_Port,
						LD1_Pin, GPIO_PIN_RESET);
					}
					device.measurements_counter++;
				}
			} else {
				HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
			}
		} else
			osDelay(10);

		if (deviceState.action == ACTION_UPDATE || HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin)) {
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
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_Base_Start(&htim6);
	for (;;) {

		if (dataBusyMutexHandle != NULL) {
			if (deviceState.action == ACTION_GET_D) {
				uint32_t event_flag = osEventFlagsWait(data_evnt_id, MSG_COLLECT, osFlagsWaitAny, osWaitForever);
				if (event_flag == MSG_COLLECT) {
					uint32_t adc_dma_displ_data_length = sizeof(data_displ) / sizeof(data_displ[0]);
					uint32_t adc_dma_accel_data_length = sizeof(data_accel) / sizeof(data_accel[0]);
					uint32_t adc_dma_temp_data_length = sizeof(HATS_temp) / sizeof(HATS_temp[0]);
					clean_buff(sensorsDataBuffer, sensorsData_attributes.mq_size);
					clean_buff(data_displ, adc_dma_displ_data_length);
					clean_buff(data_accel, adc_dma_accel_data_length);

//					volatile uint32_t start_ticks = osKernelGetTickCount();
//					volatile uint32_t now_ticks = start_ticks;

					// ///collect data from ADC/// //
					HAL_ADC_Start_DMA(&hadc1, (uint32_t*) data_displ, adc_dma_displ_data_length);
					HAL_ADC_Start_DMA(&hadc3, (uint32_t*) data_accel, adc_dma_accel_data_length);

					uint32_t data_event_flag = 0;
					// awaiting while first half of the buffer will fill up
					data_event_flag = osEventFlagsWait(dataReadyEventHandle,
					DATA_DISPL_HALF_READY_FLAG | DATA_ACCEL_HALF_READY_FLAG,
					osFlagsWaitAll, 1000);
					// fill the half of the output buffer with data from sensors
					if (data_event_flag == (DATA_DISPL_HALF_READY_FLAG | DATA_ACCEL_HALF_READY_FLAG)) {
						for (uint32_t tx_buff_counter = 0, data_iter = 0; data_iter < (adc_dma_displ_data_length / 2);
								tx_buff_counter += 2, data_iter++) {
							sensorsDataBuffer[tx_buff_counter] = data_displ[data_iter] >> 8;
							sensorsDataBuffer[tx_buff_counter + 1] = data_displ[data_iter];
						}
						for (uint32_t tx_buff_counter = 8000, data_iter = 0; data_iter < (adc_dma_accel_data_length / 2);
								tx_buff_counter += 2, data_iter++) {
							sensorsDataBuffer[tx_buff_counter] = data_accel[data_iter] >> 8;
							sensorsDataBuffer[tx_buff_counter + 1] = data_accel[data_iter];
						}

						osMutexAcquire(HATSMutexHandle, osWaitForever);
						for (uint32_t tx_buff_counter = 128000, data_iter = 0; data_iter < (adc_dma_temp_data_length / 2);
								tx_buff_counter += 2, data_iter++) {
							if (HATS_temp[data_iter] < 2000) {
								sensorsDataBuffer[tx_buff_counter] = HATS_temp[data_iter] >> 8;
								sensorsDataBuffer[tx_buff_counter + 1] = HATS_temp[data_iter];
							} else {
								sensorsDataBuffer[tx_buff_counter] = 0xFF;
								sensorsDataBuffer[tx_buff_counter + 1] = 0xFF;
							}
						}
						osMutexRelease(HATSMutexHandle);
					} else {
						//TODO: ERROR_HANDLER
					}
					//awaiting while the other half of the buffer will fill up
					data_event_flag = osEventFlagsWait(dataReadyEventHandle, DATA_DISPL_READY_FLAG | DATA_ACCEL_READY_FLAG,
					osFlagsWaitAll, 1000);
					tim6_time = __HAL_TIM_GET_COUNTER(&htim6);
					// fill the other half of the output buffer with data from sensors
					if (data_event_flag == (DATA_DISPL_READY_FLAG | DATA_ACCEL_READY_FLAG)) {
						uint16_t displ_buf_offset = adc_dma_displ_data_length / 2;
						uint16_t accel_buf_offset = adc_dma_accel_data_length / 2;
						uint16_t temperature_buf_offset = adc_dma_temp_data_length / 2;
						for (uint32_t tx_buff_counter = displ_buf_offset * 2, data_iter = displ_buf_offset; data_iter < adc_dma_displ_data_length;
								tx_buff_counter += 2, data_iter++) {
							sensorsDataBuffer[tx_buff_counter] = data_displ[data_iter] >> 8;
							sensorsDataBuffer[tx_buff_counter + 1] = data_displ[data_iter];
						}
						for (uint32_t tx_buff_counter = 8000 + accel_buf_offset * 2, data_iter = accel_buf_offset;
								data_iter < adc_dma_accel_data_length; tx_buff_counter += 2, data_iter++) {
							sensorsDataBuffer[tx_buff_counter] = data_accel[data_iter] >> 8;
							sensorsDataBuffer[tx_buff_counter + 1] = data_accel[data_iter];
						}

						osMutexAcquire(HATSMutexHandle, 500);
						for (uint32_t tx_buff_counter = 128000 + temperature_buf_offset * 2, data_iter = temperature_buf_offset;
								data_iter < adc_dma_temp_data_length; tx_buff_counter += 2, data_iter++) {
							if ((float) HATS_temp[data_iter] / 10 < 200.f) {
								sensorsDataBuffer[tx_buff_counter] = HATS_temp[data_iter] >> 8;
								sensorsDataBuffer[tx_buff_counter + 1] = HATS_temp[data_iter];
							} else {
								sensorsDataBuffer[tx_buff_counter] = 0xFF;
								sensorsDataBuffer[tx_buff_counter + 1] = 0xFF;
							}
						}
						osMutexRelease(HATSMutexHandle);
					} else {
						//TODO: ERROR_HANDLER
					}
//					now_ticks = osKernelGetTickCount();
//					now_ticks -= start_ticks;
					osEventFlagsSet(data_evnt_id, MSG_DATA_READY);
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
	/* Infinite loop */
	Max31865_init(&max_1, &hspi4, SPI_TCS1_GPIO_Port, SPI_TCS1_Pin, 2, 50);
	Max31865_init(&max_2, &hspi4, SPI_TCS2_GPIO_Port, SPI_TCS2_Pin, 2, 50);

	float temp = 0, temp2 = 0;
	osStatus_t res;
	for (;;) {
		Max31865_readTempC(&max_1, &temp);
		Max31865_readTempC(&max_2, &temp2);
		res = osMutexAcquire(HATSMutexHandle, 1000);
		if (res == osOK) {
			HATS_temp[0] = (float) temp * 10;
			HATS_temp[1] = (float) temp2 * 10;
//			HATS_new_data = 1;
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

//static void get_adc_data(ADC_HandleTypeDef *adc, uint16_t *out_buffer, uint32_t length) {
//	uint16_t data[length];
//	uint16_t temp_data[length];
//	clean_buff(temp_data, length);
//	HAL_ADC_Start_DMA(adc, (uint32_t*) data, length);
//
//	for (uint8_t i = 0; i < 10; i++) {
//		osSemaphoreAcquire(DMA2BusySemHandle, osWaitForever);
//		if (osSemaphoreAcquire(DMA2BusySemHandle, 1000) < 0) {
//			for (uint8_t j = 0; j < length; j++)
//				temp_data[j] += 0;
//		} else {
//			for (uint8_t j = 0; j < length; j++)
//				temp_data[j] += data[j];
//		}
//		osSemaphoreRelease(DMA2BusySemHandle);
//	}
//	for (uint8_t i = 0; i < length; i++)
//		out_buffer[i] = temp_data[i] / 10;
//
//	HAL_ADC_Stop_DMA(adc);
//}

void receiveDataTask(void *arg) {
	uint32_t recv_length = 0;
	char recv_buffer[20] = { 0 };

	for (;;) {
		recv_length = recv(client->soc, recv_buffer, sizeof(recv_buffer), 0);
		if (recv_length > 0 && recv_length < 0xffffffff) {
			pdataPacket_t packet = (pdataPacket_t) recv_buffer;
			switch (packet->action) {
			case ACTION_GET_D:
				deviceState.action = ACTION_GET_D;
				device.device_status = DEVICE_STATUS_COLLECT_DATA;
				break;
			case ACTION_GET_S:
				deviceState.action = ACTION_GET_S;
//				device.device_status = DEVICE_STATUS_COLLECT_DATA;
				break;
			case ACTION_STOP:
				deviceState.action = ACTION_STOP;
				device.device_status = DEVICE_STATUS_AWAITING;
				shutdown(client->soc, SHUT_RDWR);
				osDelay(1000);
				close(client->soc);
				osDelay(1000);
				client->soc = 0xffffffff;
//				prepare_to_close_connection();

				for (uint8_t i = 0; i < 120 /*9000 ms / 500 ms*/; i++) {
					HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
					osDelay(500);
				}
				connect_to_server((socketClient_t*) client);
				break;
			case ACTION_UPDATE:
				deviceState.action = ACTION_UPDATE;
				prepare_to_close_connection();
				break;
			case ACTION_ACK:
				HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
				osEventFlagsSet(data_evnt_id, MSG_PACKET_HAS_BEEN_SENT);
				break;
			default:
				break;
			}
			recv_buffer[0] = 0;
		}
	}
}

void connect_to_server(socketClient_t *client) {
	struct sockaddr_in dest_addr, local_addr;

	char deviceID[13];
	deviceGetID(&device, deviceID, 13);
	memset(client, 0, sizeof(socketClient_t));

	client->soc = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

	if (client->soc < 0) {
		while (1) {
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			osDelay(250);
		}
	} else {
		memset(&local_addr, 0, sizeof(struct sockaddr_in));
		local_addr.sin_family = AF_INET;
		local_addr.sin_port = htons(SERVER_PORT);
		local_addr.sin_addr.s_addr = INADDR_ANY;
		int8_t connection_status = -1;
		while (connection_status < 0) {
			if (bind(client->soc, (struct sockaddr *)&local_addr, sizeof(struct sockaddr_in)) == 0) {
				memset(&dest_addr, 0, sizeof(struct sockaddr_in));
				dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
				dest_addr.sin_family = AF_INET;
				dest_addr.sin_port = htons(SERVER_PORT);

				ip4addr_aton(FILESERVER_ADDRESS, (ip4_addr_t*) &dest_addr.sin_addr);

				connection_status = connect(client->soc, (struct sockaddr* )&dest_addr, sizeof(struct sockaddr_in));

				if (connection_status < 0) {
					HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
					shutdown(client->soc, SHUT_RDWR);
					osDelay(500);
					HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
					close(client->soc);
					osDelay(500);
					for (uint8_t i = 0; i < 18 /*9000 ms / 500 ms*/; i++) {
						HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
						osDelay(500);
					}
					client->soc = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
				} else {
					uint32_t opt = 10;
					int32_t ret = lwip_setsockopt(client->soc, SOL_SOCKET, SO_RCVTIMEO, &opt, sizeof(int));
					if (!ret) {
						HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
						osDelay(1000);
					}
					if (receiveDataHandle != NULL) {
						return;
					}
					receiveDataHandle = osThreadNew(receiveDataTask, NULL, &receiveData_attributes);
				}
			} else {
				for (uint8_t i = 0; i < 100; i++) {
					HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
					osDelay(250);
				}
				prepare_to_close_connection();
				client->soc = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
			}
		}
	}
}

static void prepare_to_close_connection(void) {
	shutdown(client->soc, SHUT_RDWR);
	osDelay(1000);
	close(client->soc);
	osDelay(1000);
	client->soc = 0xffffffff;
}

void decode_packet(char *in_data, uint32_t length) {
	switch (in_data[0]) {
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
		break;
	case ACTION_ACK:
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		osEventFlagsSet(data_evnt_id, MSG_PACKET_HAS_BEEN_SENT);
		break;
	default:
		break;
	}
	in_data[0] = 0;
}

////////////////display section/////////////////////////

//init eink-display and return stack for a display RAM
uint8_t screen_init(void) {
	if (screenBusyMutexHandle != NULL) {
		osStatus_t res = osMutexAcquire(screenBusyMutexHandle,
		osWaitForever);
		if (res == osOK) {
			DEV_Module_Init();
			EPD_2IN9BC_Init();
			EPD_2IN9BC_Clear();

			UWORD image_size = ((EPD_2IN9BC_WIDTH % 8 == 0) ? (EPD_2IN9BC_WIDTH / 8) : (EPD_2IN9BC_WIDTH / 8 + 1)) * EPD_2IN9BC_HEIGHT;
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

//return 96-bit unique ID of MCU in an char array
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

/////////DEBUG///////
volatile uint32_t half_cplt_counter = 0;
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *adc) {
	if (adc->Instance == hadc1.Instance) {
		osEventFlagsSet(dataReadyEventHandle, DATA_DISPL_HALF_READY_FLAG);
		osEventFlagsClear(dataReadyEventHandle, DATA_DISPL_READY_FLAG);
	} else if (adc->Instance == hadc3.Instance) {
		osEventFlagsSet(dataReadyEventHandle, DATA_ACCEL_HALF_READY_FLAG);
		osEventFlagsClear(dataReadyEventHandle, DATA_ACCEL_READY_FLAG);
	}
	half_cplt_counter++;
}

/////////DEBUG///////
volatile uint32_t conv_cplt_counter = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc) {
//	osSemaphoreRelease(DMA2BusySemHandle);
	if (adc->Instance == hadc1.Instance) {
		osEventFlagsSet(dataReadyEventHandle, DATA_DISPL_READY_FLAG);
		osEventFlagsClear(dataReadyEventHandle, DATA_DISPL_HALF_READY_FLAG);
		HAL_ADC_Stop_DMA(&hadc1);
	} else if (adc->Instance == hadc3.Instance) {
		osEventFlagsSet(dataReadyEventHandle, DATA_ACCEL_READY_FLAG);
		osEventFlagsClear(dataReadyEventHandle, DATA_ACCEL_HALF_READY_FLAG);
		HAL_ADC_Stop_DMA(&hadc3);
	}
	conv_cplt_counter++;
}

volatile uint32_t adc_overrun_error = 0;
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adc) {
	if (adc->Instance == hadc1.Instance) {
		adc_overrun_error |= 1;
	} else if (adc->Instance == hadc3.Instance) {
		adc_overrun_error |= 2;
	}
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
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) LOAD_BOOTLOADER_ADDRESS, 1);
	HAL_FLASH_Lock();
	NVIC_SystemReset();
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
