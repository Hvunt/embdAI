/*
 * device.h
 *
 *  Created on: Feb 20, 2020
 *      Author: hvunt
 */

#include <stdint.h>

#ifndef INC_DEVICE_H_
#define INC_DEVICE_H_

#define T_SENS_COUNT_REG	(0x03)
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
	uint8_t t_sens_count;			//0..2
	uint8_t dis_sens_count;			//0..4
	uint16_t time_interval;			//1..10000 uS
	uint8_t sd_card_record;			//0..3
} DeviceSettings_t;

typedef struct Device{
//	DeviceSettings_t settings;

	uint16_t data;					//data for settings
	uint8_t setting;				//if GET SETTINGS
	uint8_t sub_action;				// DATA OR SETTINGS
	uint8_t action;					// RUN / STOP / GET / SET
} Device_t;

uint16_t getSetting(DeviceSettings_t * device, uint8_t Reg);
void setSettings(DeviceSettings_t * device, uint8_t Reg, uint16_t data);

#endif /* INC_DEVICE_H_ */
