/*
 * MAX9924_driver.c
 *
 *  Created on: 27 de jan. de 2026
 *      Author: Matheus Markies
 */

#include "MAX9924_driver.h"
#include "engine_control.h"
#include "stdio.h"
#include "string.h"

VR_Sensor_t ckp_sensor;  // Sensor de virabrequim (CKP)
VR_Sensor_t cmp_sensor;  // Sensor de comando (CMP)

/**
 * @brief Inicializa os sensores VR (CKP e CMP)
 */
HAL_StatusTypeDef VR_Init(uint32_t ckp_pulses_per_rev, uint32_t ckp_tooths,
		uint32_t cmp_pulses_per_rev, uint32_t cmp_tooths, uint32_t timeout_ms) {
	// Inicializa sensor CKP (Crankshaft)
	memset(&ckp_sensor, 0, sizeof(VR_Sensor_t));
	ckp_sensor.gpio_port = CKP_GPIO_Port;
	ckp_sensor.gpio_pin = CKP_Pin;
	ckp_sensor.type = SENSOR_CKP;
	ckp_sensor.tooths = ckp_tooths;
	ckp_sensor.pulse_count_per_rev = ckp_pulses_per_rev;
	ckp_sensor.timeout_ms = timeout_ms;

	// Inicializa sensor CMP (Camshaft)
	memset(&cmp_sensor, 0, sizeof(VR_Sensor_t));
	cmp_sensor.gpio_port = CMP_GPIO_Port;
	cmp_sensor.gpio_pin = CMP_Pin;
	cmp_sensor.type = SENSOR_CMP;
	cmp_sensor.tooths = cmp_tooths;
	cmp_sensor.pulse_count_per_rev = cmp_pulses_per_rev;
	cmp_sensor.timeout_ms = timeout_ms;

	return HAL_OK;
}

/**
 * @brief Calcula delta T
 */
static uint32_t VR_CalculateDeltaT(uint32_t current, uint32_t previous) {
	if (current >= previous) {
		return current - previous;
	} else {
		// Overflow do timer
		return (0xFFFFFFFF - previous) + current + 1;
	}
}

float alpha;

void VR_InputCaptureCallback(VR_Sensor_Type_t type) {
	uint32_t current_time = 0;
	VR_Sensor_t temp;

	uint8_t is_largest_tooth = 0;

	if (type == SENSOR_CKP) {
		current_time = HAL_TIM_ReadCapturedValue(&htim5, CKP_CHANNEL);
		temp = ckp_sensor;
	} else {
		current_time = HAL_TIM_ReadCapturedValue(&htim5, CMP_CHANNEL);
		temp = cmp_sensor;
	}

	if (temp.rpm < 1000)
	    alpha = 0.08f;
	else if (temp.rpm < 3000)
	    alpha = 0.12f;
	else if (temp.rpm < 10000)
	    alpha = 0.2f;
	else alpha = 0.25f;

	temp.current_edge_time = current_time;
	temp.last_period = temp.period;

	uint32_t delta = VR_CalculateDeltaT(current_time, temp.last_edge_time);
	temp.filtered_delta_us = temp.filtered_delta_us + alpha * ((float)delta - temp.filtered_delta_us); //EMA (Exponential Moving Average) PRECISA DIVIDIR ENTRE CMP E CKP

	if (delta < 60) {  // < 1/15000s = > 15kHz 14000/60 * tooths Hz
		return;
	}

	float ratio = (float) delta / (float) temp.period;

	uint32_t expected_count = temp.is_first_rev ? (temp.tooths * 2) : temp.tooths;
	if ((ratio >= LARGEST_RATIO || temp.pulse_count >= expected_count) && temp.pulse_count > 1) {
		if (temp.isSync) {
			temp.revolution_count += 1;

			is_largest_tooth = 1;
		} else
			temp.isSync = true;

		if (temp.is_first_rev == 0) {
			temp.pulse_count = 0;
			temp.is_first_rev = 1;
		} else
			temp.is_first_rev = 0;
	} else{
		temp.frequency_hz = 1e6f/temp.filtered_delta_us;
		temp.rpm = 60.0f * temp.frequency_hz;
	}

	temp.last_edge_time = current_time;
	temp.period = delta;  //us

	temp.pulse_count+=1;

	if (type == SENSOR_CKP) {
		ckp_sensor = temp;
		ENGINE_CKP_Callback(ckp_sensor);
	} else {
		cmp_sensor = temp;
		//ENGINE_CMP_Callback(cmp_sensor);
	}
}

/**
 * @brief Retorna nome do sensor como string
 */
const char* VR_GetSensorName(VR_Sensor_Type_t type) {
	switch (type) {
	case SENSOR_CKP:
		return "CKP";
	case SENSOR_CMP:
		return "CMP";
	default:
		return "UNKNOWN";
	}
}
