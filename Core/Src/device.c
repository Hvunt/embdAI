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
	deviceSettings->la_t_sens_count = 2;
	deviceSettings->ha_t_sens_count = 2;
	deviceSettings->time_interval = 100;
	deviceSettings->use_mic = 0;
}

uint16_t deviceGetSetting(DeviceSettings_t *deviceSettings, uint8_t Reg) {
	uint16_t result = 0xFF;
	switch (Reg) {
	case LA_T_SENS_COUNT_REG:
		result = deviceSettings->la_t_sens_count;
		break;
	case HA_T_SENS_COUNT_REG:
		result = deviceSettings->la_t_sens_count;
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
	case LA_T_SENS_COUNT_REG:
		deviceSettings->la_t_sens_count = data;
		break;
	case HA_T_SENS_COUNT_REG:
		deviceSettings->ha_t_sens_count = data;
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

void deviceGetID(DeviceSettings_t *device, char *_id, uint8_t len){
	if (len > 11)
		strcpy(_id, device->deviceID);
}
void deviceSetID(DeviceSettings_t *device, char *_id, uint8_t len){
	if (len > 11){
		strcpy(device->deviceID, _id);
		device->deviceID[12] = '\0';
	}
}

