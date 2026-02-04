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

	uint8_t is_largest_tooth = 0;

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

    float ratio = (float)delta/(float)temp.period;

    if(ratio >= 2.5f){

    	if(temp.isSync){
        	temp.pulse_count_per_rev = temp.pulse_count;
        	temp.revolution_count += 1;

        	is_largest_tooth = 1;
    	}else
    		temp.isSync = true;

    	if(temp.is_first_rev = 0){
    		temp.pulse_count = 0;
    		temp.is_first_rev = 1;
    	}else temp.is_first_rev = 0;

    }else
    	temp.frequency_hz = 1.0f/(float)delta * 1000000.0f;

    temp.last_edge_time = current_time;
	temp.period = delta;//us

	if(type == SENSOR_CKP){
		ckp_sensor = temp;
		ckp_sensor.pulse_count+=1;

		if(is_largest_tooth)
			ENGINE_CKP_Callback(ckp_sensor);

	}else{
		cmp_sensor = temp;
		cmp_sensor.pulse_count+=1;

		if(is_largest_tooth)
			ENGINE_CMP_Callback(cmp_sensor);
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
