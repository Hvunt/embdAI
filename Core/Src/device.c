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
	deviceSettings->voltage_sens_count = 4;
	deviceSettings->ha_t_sens_count = 2;
	deviceSettings->time_interval = 50;
	deviceSettings->use_mic = 0;
	deviceSettings->measurements_counter = 1;
}

uint16_t deviceGetSetting(DeviceSettings_t *deviceSettings, uint8_t Reg) {
	uint16_t result = 0xFF;
	switch (Reg) {
	case HA_T_SENS_COUNT_REG:
		result = deviceSettings->ha_t_sens_count;
		break;
	case VLTG_SENS_COUNT_REG:
		result = deviceSettings->voltage_sens_count;
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
	case HA_T_SENS_COUNT_REG:
		deviceSettings->ha_t_sens_count = data;
		break;
	case VLTG_SENS_COUNT_REG:
		deviceSettings->voltage_sens_count = data;
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

