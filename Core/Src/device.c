/*
 * device.c
 *
 *  Created on: Feb 21, 2020
 *      Author: hvunt
 */

#include "device.h"

uint16_t getSetting(DeviceSettings_t *device, uint8_t Reg) {
	uint16_t result = 0xFF;
	switch (Reg) {
	case T_SENS_COUNT_REG:
		result = device->t_sens_count;
		break;
	case DIS_SENS_COUNT_REG:
		result = device->dis_sens_count;
		break;
	case TIME_INTERVAL_REG:
		result = device->time_interval;
		break;
	case SD_CARD_RECORD_REG:
		result = device->sd_card_record;
		break;
	}
	return result;
}

void setSettings(DeviceSettings_t *device, uint8_t Reg, uint16_t data) {
	switch (Reg) {
	case T_SENS_COUNT_REG:
		device->t_sens_count = data;
		break;
	case DIS_SENS_COUNT_REG:
		device->dis_sens_count = data;
		break;
	case TIME_INTERVAL_REG:
		device->time_interval = data;
		break;
	case SD_CARD_RECORD_REG:
		device->sd_card_record = data;
		break;
	}
}
