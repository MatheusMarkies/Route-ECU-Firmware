/*
 * engine_control.c
 *
 *  Created on: 28 de jan. de 2026
 *      Author: Matheus Markies
 */

#include "engine_control.h"
#include "stdio.h"
#include "string.h"

Injector_State_t injector_state[4] = { 0 };
ENGINE_Shafts_t shafts;

const uint32_t injector[4] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4
};

const uint32_t ignition[4] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4
};


static uint32_t ENGINE_CalculateDeltaT(uint32_t current, uint32_t previous) {
	if (current >= previous) {
		return current - previous;
	} else {
		// Overflow do timer
		return (0xFFFFFFFF - previous) + current + 1;
	}
}

static float ENGINE_CalculateAngle(uint32_t current, VR_Sensor_t sensor) {
	float angular_velocity = 0;
	float fixed_angle = 0;

	if (sensor.frequency_hz <= 0.0f || !sensor.isSync)
		return 0.0f;

	if (sensor.frequency_hz > 0) {
		fixed_angle = (2.0f * M_PI) / sensor.tooths * sensor.pulse_count; //angulo por pulso * pulsos
		angular_velocity = 2.0f * M_PI * sensor.frequency_hz;

		float dt_seconds = (float) ENGINE_CalculateDeltaT(current,
				sensor.current_edge_time) * MICROS_TO_SECONDS;

		return fixed_angle + angular_velocity * dt_seconds;
	}
	return 0.0f;
}

void ENGINE_VR_OutputCompareCallback(uint32_t current_time,
		VR_Sensor_t ckp_sensor, VR_Sensor_t cmp_sensor) {
	shafts.crakshaft_angle = ENGINE_CalculateAngle(current_time, ckp_sensor);
	shafts.camshaft_angle = ENGINE_CalculateAngle(current_time, cmp_sensor);
}

void ENGINE_Injector_FireNow(uint8_t inj, uint16_t pulse_us) {
	if (pulse_us < 500)
		pulse_us = 500;
	if (pulse_us > 20000)
		pulse_us = 20000;

	uint32_t channel = injector[inj];
	__HAL_TIM_SET_COMPARE(&htim1, channel, pulse_us);
	__HAL_TIM_SET_COUNTER(&htim1, 0);

	HAL_TIM_PWM_Start(&htim1, channel);
	injector_state[inj].pulse_start_time = __HAL_TIM_GET_COUNTER(&htim1);
	injector_state[inj].isArmed = 0;
	injector_state[inj].isActive = 1;
}

void ENGINE_Injector_Schedule(uint8_t inj, uint16_t pulse_us) {
	if (pulse_us < 500)
		pulse_us = 500;
	if (pulse_us > 20000)
		pulse_us = 20000;

	injector_state[inj].pulse_width_us = pulse_us;
	injector_state[inj].isArmed = 1;

	uint32_t channel = injector[inj];
	__HAL_TIM_SET_COMPARE(&htim1, channel, pulse_us);
}

void ENGINE_Injector_Trigger(uint8_t inj) {
	if (!injector_state[inj].isArmed) {
		return;
	}

	uint32_t channel = injector[inj];

	__HAL_TIM_SET_COUNTER(&htim1, 0);

	HAL_TIM_PWM_Start(&htim1, channel);
	injector_state[inj].pulse_start_time = __HAL_TIM_GET_COUNTER(&htim1);
	injector_state[inj].isArmed = 0;
	injector_state[inj].isActive = 1;
}

void ENGINE_Injector_StopAll(void) {
	for (int i = 0; i < 4; i++) {
		HAL_TIM_PWM_Stop(&htim1, injector[i]);
		injector_state[i].isArmed = 0;
	}
}

void ENGINE_Injector_OutputCompareCallback(uint32_t current_time) {
	for (int i = 0; i < 4; i++) {
		if (injector_state[i].isActive) {

			uint32_t elapsed_time = ENGINE_CalculateDeltaT(current_time,
					injector_state[i].pulse_start_time);
			if (elapsed_time >= injector_state[i].pulse_width_us) {
				HAL_TIM_PWM_Stop(&htim1, injector[i]);
				injector_state[i].isActive = 0;
				injector_state[i].pulse_start_time = 0;
				injector_state[i].pulse_width_us = 0;
			}

		}
	}
}


void ENGINE_Ignition_OutputCompareCallback(uint32_t current_time) {

}
