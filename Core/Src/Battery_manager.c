/*
 * Battery_manager.c
 *
 *  Created on: 28 de jan. de 2026
 *      Author: Matheus Markies
 */

#include "Battery_manager.h"
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;
Battery_t battery = { .voltage = 0.0f, .raw_adc = 0, .linear_cte =
0.0f, .lvL = { .voltage = 0.0f, .raw_adc = 0 }, .lvH = {
		.voltage = 12.0f, .raw_adc = 0 } };

void BATTERY_Init(void) {
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)
			!= HAL_OK) {
		Error_Handler();
	}
}

uint16_t BATTERY_ReadRaw(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	uint16_t adc_value = 0;

	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		adc_value = HAL_ADC_GetValue(&hadc1);
	}

	HAL_ADC_Stop(&hadc1);

	return adc_value;
}

float BATTERY_ReadVoltage(void) {
	uint32_t sum = 0;

	for (int i = 0; i < BATTERY_SAMPLES; i++) {
		sum += BATTERY_ReadRaw();
		HAL_Delay(2);
	}

	uint16_t adc_avg = sum / BATTERY_SAMPLES;
	battery.raw_adc = adc_avg;

	if (battery.linear_cte <= 0.0f) {
	    battery.linear_cte = 18.81f / 4095.0f;

	    battery.lvH.voltage = 18.81f;
	    battery.lvH.raw_adc = 4095;

	    battery.lvL.voltage = 0.0f;
	    battery.lvL.raw_adc = 0;
	}

	float battery_voltage = battery.lvL.voltage
			+ (battery.linear_cte * (float) (adc_avg - battery.lvL.raw_adc));

	if (battery_voltage < MIN_BATTERY_VOLTAGE) {
		battery_voltage = MIN_BATTERY_VOLTAGE;
	} else if (battery_voltage > MAX_BATTERY_VOLTAGE) {
		battery_voltage = MAX_BATTERY_VOLTAGE;
	}

	battery.voltage = battery_voltage;

	battery.voltage = battery_voltage;

	return battery_voltage;
}

void BATTERY_ReadVoltageFiltered(void) {
	uint32_t sum = 0;

	for (int i = 0; i < BATTERY_SAMPLES; i++) {
		sum += BATTERY_ReadRaw();
		HAL_Delay(2);
	}

	uint16_t adc_avg = sum / BATTERY_SAMPLES;
	battery.raw_adc = adc_avg;

	if (battery.linear_cte <= 0.0f) {
	    battery.linear_cte = 18.81f / 4095.0f;

	    battery.lvH.voltage = 18.81f;
	    battery.lvH.raw_adc = 4095;

	    battery.lvL.voltage = 0.0f;
	    battery.lvL.raw_adc = 0;
	}

	float battery_voltage = battery.lvL.voltage
			+ (battery.linear_cte * (float) (adc_avg - battery.lvL.raw_adc));

	if (battery_voltage < MIN_BATTERY_VOLTAGE) {
		battery_voltage = MIN_BATTERY_VOLTAGE;
	} else if (battery_voltage > MAX_BATTERY_VOLTAGE) {
		battery_voltage = MAX_BATTERY_VOLTAGE;
	}

	battery.voltage = battery_voltage;

	battery.voltage = battery_voltage;
}

void BATTERY_Calibrate(void) {
	uint16_t sum = 0;

	for (int i = 0; i < 20; i++) {
		sum += BATTERY_ReadRaw();
		HAL_Delay(1);
	}

	battery.lvL.voltage = 0.0f;
	battery.lvL.raw_adc = (uint16_t) (sum / 20.0f);
}

void BATTERY_Calibrate_LV(float value) {
	battery.lvH.voltage = value;

	uint32_t sum = 0;

	for (int i = 0; i < 20; i++) {
		uint16_t raw = BATTERY_ReadRaw();
		sum += raw;
		HAL_Delay(1);
	}

	uint16_t adc_raw = (uint16_t) (sum / 20.0f);
	battery.lvH.raw_adc = adc_raw;

	float delta_voltage = battery.lvH.voltage - battery.lvL.voltage;
	float delta_adc = (float) (battery.lvH.raw_adc - battery.lvL.raw_adc);

	printf("[BATTERY] Delta voltage: %.4f\r\n", delta_voltage);
	printf("[BATTERY] Delta ADC: %.4f\r\n", delta_adc);

	if (delta_adc > 0) {
		battery.linear_cte = delta_voltage / delta_adc;
		printf("[BATTERY] linear_cte calculado: %.6f\r\n", battery.linear_cte);
	} else {
		printf("[BATTERY] ERRO: Delta ADC <= 0!\r\n");
	}

}
