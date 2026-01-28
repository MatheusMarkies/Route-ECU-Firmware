/*
 * MAX9924_driver.c
 *
 *  Created on: 27 de jan. de 2026
 *      Author: Matheus Markies
 */

#include "MAX9924_driver.h"
#include "stdio.h"
#include "string.h"

VR_Sensor_t ckp_sensor;  // Sensor de virabrequim (CKP)
VR_Sensor_t cmp_sensor;  // Sensor de comando (CMP)

/**
 * @brief Inicializa os sensores VR (CKP e CMP)
 */
HAL_StatusTypeDef VR_Init(uint32_t ckp_pulses_per_rev, uint32_t cmp_pulses_per_rev, uint32_t timeout_ms) {
    // Inicializa sensor CKP (Crankshaft)
    memset(&ckp_sensor, 0, sizeof(VR_Sensor_t));
    ckp_sensor.gpio_port = CKP_GPIO_Port;
    ckp_sensor.gpio_pin = CKP_Pin;
    ckp_sensor.type = SENSOR_CKP;
    ckp_sensor.pulse_count_per_rev = ckp_pulses_per_rev;
    ckp_sensor.timeout_ms = timeout_ms;

    // Inicializa sensor CMP (Camshaft)
    memset(&cmp_sensor, 0, sizeof(VR_Sensor_t));
    cmp_sensor.gpio_port = CMP_GPIO_Port;
    cmp_sensor.gpio_pin = CMP_Pin;
    cmp_sensor.type = SENSOR_CMP;
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

void VR_InputCaptureCallback(VR_Sensor_Type_t type){
	uint32_t current_time = 0;
	VR_Sensor_t temp;

	if(type == SENSOR_CKP){
		current_time = HAL_TIM_ReadCapturedValue(&htim5, CKP_CHANNEL);
		ckp_sensor.current_edge_time = current_time;
		temp = ckp_sensor;
	}else{
		current_time = HAL_TIM_ReadCapturedValue(&htim5, CMP_CHANNEL);
		cmp_sensor.current_edge_time = current_time;
		temp = cmp_sensor;
	}

	uint32_t delta = VR_CalculateDeltaT(current_time, temp.last_edge_time);

    if (delta < 60) {  // < 1/15000s = > 15kHz 14000/60 * tooths Hz
        return;
    }

    uint32_t rpm = 1.0f/(float)delta * 1000000.0f;
    rpm *= 60.0f/(float)temp.pulse_count_per_rev;

    float ratio = delta/temp.period;

    if(ratio >= 2.5f){
    	temp.isSync = true;
    	temp.pulse_count = 0;
    	temp.revolution_count += 1;
    }

	if(type == SENSOR_CKP){
		ckp_sensor = temp;

		ckp_sensor.pulse_count+=1;
		ckp_sensor.last_edge_time = current_time;
		ckp_sensor.period = delta;
	}else{
		cmp_sensor = temp;

		cmp_sensor.pulse_count+=1;
		cmp_sensor.last_edge_time = current_time;
		cmp_sensor.period = delta;
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

/**
 * @brief Gera JSON com dados dos sensores VR
 */
int VR_GenerateJSON(char *buffer, size_t buffer_size) {
    if (buffer == NULL || buffer_size == 0) {
        return -1;
    }

    int offset = snprintf(buffer, buffer_size,
        "VRSENSORS:{\"ckp\":{\"rpm\":%.2f,\"freq_hz\":%.2f,\"pulses\":%lu,"
        "\"revolutions\":%lu,\"period_ms\":%lu,\"valid\":%s},"
        "\"cmp\":{\"rpm\":%.2f,\"freq_hz\":%.2f,\"pulses\":%lu,"
        "\"revolutions\":%lu,\"period_ms\":%lu,\"valid\":%s}}\r\n",
        ckp_sensor.rpm,
        ckp_sensor.frequency_hz,
        ckp_sensor.pulse_count,
        ckp_sensor.revolution_count,
        ckp_sensor.period,

        cmp_sensor.rpm,
        cmp_sensor.frequency_hz,
        cmp_sensor.pulse_count,
        cmp_sensor.revolution_count,
        cmp_sensor.period
    );

    return (offset < buffer_size) ? offset : -1;
}
