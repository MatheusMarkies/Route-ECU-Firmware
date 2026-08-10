/*
 * MAX9924_driver.c
 *
 *  Created on: 27 de jan. de 2026
 *      Author: Matheus Markies
 */

#include "MAX9924_driver.h"
#include "engine_control.h"
#include "Battery_manager.h"
#include "MAP_sensor.h"
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
	ckp_sensor.is_first_rev = 1;

	// Inicializa sensor CMP (Camshaft)
	memset(&cmp_sensor, 0, sizeof(VR_Sensor_t));
	cmp_sensor.gpio_port = CMP_GPIO_Port;
	cmp_sensor.gpio_pin = CMP_Pin;
	cmp_sensor.type = SENSOR_CMP;
	cmp_sensor.tooths = cmp_tooths;
	cmp_sensor.pulse_count_per_rev = cmp_pulses_per_rev;
	cmp_sensor.timeout_ms = timeout_ms;
	cmp_sensor.is_first_rev = 1;

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

	if (type == SENSOR_CKP) {
		current_time = HAL_TIM_ReadCapturedValue(&htim5, CKP_CHANNEL);
		temp = ckp_sensor;
	} else {
		current_time = HAL_TIM_ReadCapturedValue(&htim5, CMP_CHANNEL);
		temp = cmp_sensor;
	}

	temp.current_edge_time = current_time;

	uint32_t delta = VR_CalculateDeltaT(temp.current_edge_time,
			temp.last_edge_time);
	uint32_t ratio =
			(temp.filtered_delta_us == 0) ?
					0 : (delta / temp.filtered_delta_us);

	temp.filtered_delta_us = delta;

	if(ratio >= LARGEST_RATIO || (DEBUG_CKP == 1 && temp.pulse_count > 57)){
		temp.pulse_count = 0;
		temp.elapsed_time = 0;
		temp.isSync = 1;

		temp.is_first_rev = !temp.is_first_rev;
	}else{
		temp.pulse_count += 1;
		temp.elapsed_time += (float)delta / 1e6f;

		float revolution_freq = (temp.pulse_count / (temp.elapsed_time * temp.pulse_count_per_rev));

		temp.frequency_hz = (revolution_freq * temp.pulse_count_per_rev);

		temp.last_period = temp.period;

		temp.period = 1.0f / revolution_freq;
		temp.rpm = 60.0f * revolution_freq;
	}

	temp.last_edge_time = current_time;

	if (type == SENSOR_CKP) {
		ckp_sensor = temp;

		MAP_OnCrankTooth();
		BATTERY_OnCrankTooth();

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
