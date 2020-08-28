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

#define LA_T_SENS_COUNT_REG	(0x03)
#define HA_T_SENS_COUNT_REG	(0x04)
#define DIS_SENS_COUNT_REG 	(0x09)
#define TIME_INTERVAL_REG 	(0x0C)
#define SD_CARD_RECORD_REG 	(0x10)

#define ACTION_RUN			(0xBD)
#define ACTION_STOP			(0xA5)
#define ACTION_GET			(0x55)
#define ACTION_SET			(0xAA)

#define ACTION_DATA			(0xCA)
#define ACTION_SETTINGS		(0x59)

enum {
	SD_CARD_RECORD_STOP = 0x00,
	SD_CARD_RECORD_ALL,
	SD_CARD_RECORD_RUN,
	SD_CARD_RECORD_GET
};

typedef struct DeviceSettings{
	uint8_t la_t_sens_count;		//0..2 -- Low Accuracy temperature sensors
	uint8_t ha_t_sens_count;		//0..2 -- High Accuracy temperature sensors
	uint8_t dis_sens_count;			//0..4 -- Displacement sensors
	uint16_t time_interval;			//100..10000 ms
	uint8_t sd_card_record;			//0..3 -- type of record and transmit data
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
