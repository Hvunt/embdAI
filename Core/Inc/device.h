/*
 * device.h
 *
 *  Created on: Feb 20, 2020
 *      Author: hvunt
 */

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

typedef struct DeviceSettings{
	uint8_t t_sens_count;			//0..2
	uint8_t dis_sens_count;			//0..4
	uint16_t time_interval;			//0..10000 ms
	uint8_t sd_card_record;			//0..3
} DeviceSettings_t;

typedef struct Device{
	DeviceSettings_t settings;
	uint8_t sub_action;				// DATA OR SETTINGS
	uint8_t action;					// RUN / STOP / GET / SET
} Device_t;

#endif /* INC_DEVICE_H_ */
