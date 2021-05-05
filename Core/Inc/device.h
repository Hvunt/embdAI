/*
 * device.h
 *
 *  Created on: Feb 20, 2020
 *      Author: hvunt
 */

#include <stdint.h>
#include <string.h>

#ifndef INC_DEVICE_H_
#define INC_DEVICE_H_

#define HA_T_SENS_COUNT_REG		(0x04)
#define VLTG_SENS_COUNT_REG 	(0x09)
#define TIME_INTERVAL_REG 		(0x0C)
#define SD_CARD_RECORD_REG 		(0x10)
#define MIC_REG					(0x11)

enum {
	ACTION_RUN = 0x01,
	ACTION_STOP,
	ACTION_UPDATE,
	ACTION_GET_D,
	ACTION_GET_S,
	ACTION_SET_S,
	ACTION_ACK,


	ACTION_ERR = 0xFF
};

//#define ACTION_RUN				('r')
//#define ACTION_STOP				('s')
//#define ACTION_UPDATE			('u')
//#define ACTION_GET_D			('p')
//#define ACTION_GET_S			('g')
//#define ACTION_SET				('e')
//#define ACTION_ACK				('a')

#define ACTION_DATA				(0xCA)
#define ACTION_SETTINGS			(0x59)

enum {
	SD_CARD_RECORD_STOP = 0x00,
	SD_CARD_RECORD_ALL,
	SD_CARD_RECORD_RUN,
	SD_CARD_RECORD_GET
};

enum {
	DEVICE_STATUS_AWAITING = 0,
	DEVICE_STATUS_COLLECT_DATA,
	DEVICE_STATUS_LEARNING,
	DEVICE_STATUS_MONITORING,
	DEVICE_STATUS_HW_ERROR,
	DEVICE_STATUS_NO_IP_ERROR,
	DEVICE_STATUS_NO_CONNECTION_ERROR,
};

typedef struct DeviceSettings{
	//count of used sensors
	uint8_t ha_t_sens_count;		//0..2 -- High Accuracy temperature sensors
	uint8_t voltage_sens_count;		//0..4 -- Standard analog voltage input
	uint8_t use_mic;				//use a microphone port

	//time interval between each measured data packets
	uint16_t time_interval;			//100..10000 ms
	uint16_t samples;				//samples count 15...20
	volatile uint32_t measurements_counter;

	uint8_t sd_card_record;			//0..3 -- type of the recording and transmitting the data

	uint8_t device_status;			//current device status

	char deviceID[13];				//unique device ID
} DeviceSettings_t;

typedef struct DeviceAction{
	uint16_t data;					//data for settings
	uint8_t setting;				//if GET SETTINGS
	uint8_t sub_action;				// DATA OR SETTINGS
	uint8_t action;					// RUN / STOP / GET / SET
} DeviceAction_t;

void deviceSettingsInit(DeviceSettings_t *deviceSettings);
uint16_t deviceGetSetting(DeviceSettings_t * device, uint8_t Reg);
void deviceSetSettings(DeviceSettings_t * device, uint8_t Reg, uint16_t data);
void deviceGetID(DeviceSettings_t *device, char *_id, uint8_t len);
void deviceSetID(DeviceSettings_t *device, char *_id, uint8_t len);

#endif /* INC_DEVICE_H_ */
