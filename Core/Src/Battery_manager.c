/*
 * Battery_manager.c
 *
 *  Created on: 28 de jan. de 2026
 *      Author: Matheus Markies
 */

#include "Battery_manager.h"

extern ADC_HandleTypeDef hadc1;
Battery_t battery;

uint16_t BATTERY_ReadRaw(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	uint16_t adc_value = 0;

	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {

	}

	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		adc_value = HAL_ADC_GetValue(&hadc1);
	}

	HAL_ADC_Stop(&hadc1);

	return adc_value;
}

float BATTERY_ReadVoltage(void) {
	uint16_t adc_raw = BATTERY_ReadRaw();

	float voltage_adc = (adc_raw * VREF) / ADC_MAX;
	float battery_voltage = voltage_adc * VOLTAGE_DIVIDER_FACTOR;

	return battery_voltage;
}

void BATTERY_ReadVoltageFiltered(void) {
	uint32_t sum = 0;

	for (int i = 0; i < BATTERY_SAMPLES; i++) {
		sum += BATTERY_ReadRaw();
		HAL_Delay(1);
	}

	uint16_t adc_avg = sum / BATTERY_SAMPLES;

	float voltage_adc = (adc_avg * VREF) / ADC_MAX;
	float battery_voltage = voltage_adc * VOLTAGE_DIVIDER_FACTOR;

	battery.voltage = battery_voltage;
}
