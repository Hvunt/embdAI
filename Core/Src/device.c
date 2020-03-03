/*
 * device.c
 *
 *  Created on: Feb 21, 2020
 *      Author: hvunt
 */

#include "device.h"

// Init default settings
void deviceSettingsInit(DeviceSettings_t *deviceSettings) {
	deviceSettings->sd_card_record = SD_CARD_RECORD_ALL;
	deviceSettings->dis_sens_count = 4;
	deviceSettings->t_sens_count = 2;
	deviceSettings->time_interval = 100;
}

uint16_t deviceGetSetting(DeviceSettings_t *deviceSettings, uint8_t Reg) {
	uint16_t result = 0xFF;
	switch (Reg) {
	case T_SENS_COUNT_REG:
		result = deviceSettings->t_sens_count;
		break;
	case DIS_SENS_COUNT_REG:
		result = deviceSettings->dis_sens_count;
		break;
	case TIME_INTERVAL_REG:
		result = deviceSettings->time_interval;
		break;
	case SD_CARD_RECORD_REG:
		result = deviceSettings->sd_card_record;
		break;
	}
	return result;
}

void deviceSetSettings(DeviceSettings_t *deviceSettings, uint8_t Reg,
		uint16_t data) {
	switch (Reg) {
	case T_SENS_COUNT_REG:
		deviceSettings->t_sens_count = data;
		break;
	case DIS_SENS_COUNT_REG:
		deviceSettings->dis_sens_count = data;
		break;
	case TIME_INTERVAL_REG:
		deviceSettings->time_interval = data;
		break;
	case SD_CARD_RECORD_REG:
		deviceSettings->sd_card_record = data;
		break;
	}
}
