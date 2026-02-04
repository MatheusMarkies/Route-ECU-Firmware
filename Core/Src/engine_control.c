/*
 * engine_control.c
 *
 *  Created on: 28 de jan. de 2026
 *      Author: Matheus Markies
 */

#include "engine_control.h"
#include "stdio.h"
#include "string.h"

uint8_t injector_loop_test = 0;
uint32_t last_injector_loop_tick;

uint8_t ignition_loop_test = 0;
uint32_t last_ignition_loop_tick;

Injector_State_t injector_state[4] = { 0 };
Ignition_State_t ignition_state[4] = { 0 };

ENGINE_t engine;

const uint32_t injector[4] = {
TIM_CHANNEL_1,
TIM_CHANNEL_2,
TIM_CHANNEL_3,
TIM_CHANNEL_4 };

const uint32_t ignition[4] = {
TIM_CHANNEL_1,
TIM_CHANNEL_2,
TIM_CHANNEL_3,
TIM_CHANNEL_4 };

static uint32_t ENGINE_CalculateDeltaT(uint32_t current, uint32_t previous) {
	if (current >= previous) {
		return current - previous;
	} else {
		// Overflow do timer
		return (0xFFFFFFFF - previous) + current + 1;
	}
}

static uint32_t ENGINE_CalculateDeltaTBasedOnAngle(uint32_t current_angle, uint32_t target_angle) {
	    if (target_angle >= current_angle) {
	        return target_angle - current_angle;
	    }else {
	        return (ANGLE_MAX - current_angle) + target_angle;
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

static float ENGINE_CalculateAngularVelocity(uint32_t current, VR_Sensor_t sensor) {
	float angular_velocity = 0;
	float fixed_angle = 0;

	if (sensor.frequency_hz <= 0.0f || !sensor.isSync)
		return 0.0f;

	if (sensor.frequency_hz > 0) {
		angular_velocity = 2.0f * M_PI * sensor.frequency_hz;
		return angular_velocity;
	}
	return 0.0f;
}

void ENGINE_CKP_Callback(VR_Sensor_t ckp_sensor) {
	uint32_t current_inj_time = __HAL_TIM_GET_COUNTER(&htim1);
	uint32_t current_ign_time = __HAL_TIM_GET_COUNTER(&htim4);

	uint32_t current_ckp_time = __HAL_TIM_GET_COUNTER(&htim5);

	engine.crakshaft_angular_velocity = ENGINE_CalculateAngularVelocity(current_ckp_time, ckp_sensor);
	engine.crakshaft_angle = ENGINE_CalculateAngle(current_ckp_time, ckp_sensor); //<- Zera o angulo ja que estamos no 0

}

void ENGINE_CMP_Callback(VR_Sensor_t cmp_sensor) {
	uint32_t current_inj_time = __HAL_TIM_GET_COUNTER(&htim1);
	uint32_t current_ign_time = __HAL_TIM_GET_COUNTER(&htim4);

	uint32_t current_cmp_time = __HAL_TIM_GET_COUNTER(&htim5);

	engine.camshaft_angular_velocity = ENGINE_CalculateAngularVelocity(current_cmp_time, cmp_sensor);
	engine.camshaft_angle = ENGINE_CalculateAngle(current_cmp_time, cmp_sensor); //<- Zera o angulo ja que estamos no 0
}

void ENGINE_VR_OutputCompareCallback(uint32_t current_time,
		VR_Sensor_t ckp_sensor, VR_Sensor_t cmp_sensor) {
	engine.crakshaft_angular_velocity = ENGINE_CalculateAngularVelocity(current_time, ckp_sensor);
	engine.camshaft_angular_velocity = ENGINE_CalculateAngularVelocity(current_time, cmp_sensor);

	engine.crakshaft_angle = ENGINE_CalculateAngle(current_time, ckp_sensor);
	engine.camshaft_angle = ENGINE_CalculateAngle(current_time, cmp_sensor);
}

static const uint32_t INJ_IT_FLAGS[4] = {TIM_IT_CC1, TIM_IT_CC2, TIM_IT_CC3, TIM_IT_CC4};
static const uint32_t INJ_CLR_FLAGS[4]= {TIM_FLAG_CC1, TIM_FLAG_CC2, TIM_FLAG_CC3, TIM_FLAG_CC4};

void ENGINE_Schedule_Injection(uint8_t cyl_id, uint16_t start_tick, uint16_t pulse_us) {

    uint32_t channel = injector[cyl_id];

	injector_state[cyl_id].pulse_width_us = pulse_us;
	injector_state[cyl_id].pulse_start_time = start_tick;

	injector_state[cyl_id].state = INJ_STATE_SCHEDULED;

    __HAL_TIM_SET_COMPARE(&htim1, channel, injector_state[cyl_id].pulse_start_time);

    __HAL_TIM_SET_OC_MODE(&htim1, channel, TIM_OCMODE_ACTIVE);

    __HAL_TIM_CLEAR_FLAG(&htim1, INJ_CLR_FLAGS[cyl_id]);
    __HAL_TIM_ENABLE_IT(&htim1, INJ_IT_FLAGS[cyl_id]);
}

void ENGINE_Injector_FireNow(uint8_t inj, uint16_t pulse_us) {
	if (pulse_us < 500)
		pulse_us = 500;
	if (pulse_us > 20000)
		pulse_us = 20000;

	uint32_t channel = injector[inj];

	injector_state[inj].pulse_width_us = pulse_us;
	injector_state[inj].pulse_start_time = __HAL_TIM_GET_COUNTER(&htim1);

	injector_state[inj].state = INJ_STATE_ACTIVE;

	uint32_t turn_off_cnt = injector_state[inj].pulse_start_time + injector_state[inj].pulse_width_us;

	__HAL_TIM_SET_COMPARE(&htim1, channel, turn_off_cnt);
	__HAL_TIM_SET_OC_MODE(&htim1, channel, TIM_OCMODE_FORCED_ACTIVE);

	__HAL_TIM_SET_OC_MODE(&htim1, channel, TIM_OCMODE_INACTIVE);

    __HAL_TIM_CLEAR_FLAG(&htim1, INJ_CLR_FLAGS[inj]);
    __HAL_TIM_ENABLE_IT(&htim1, INJ_IT_FLAGS[inj]);
}

void ENGINE_Injector_TestLoop(void) {
    if (!injector_loop_test) {
        return;
    }

    uint32_t current_tick = HAL_GetTick();
    if ((current_tick - last_injector_loop_tick) < 2000) {
        return;
    }

    last_injector_loop_tick = current_tick;

    for (int i = 0; i < 4; i++) {
        ENGINE_Injector_FireNow(i, (uint16_t)300000);
    }
}
void ENGINE_Injector_StopAll(void) {
	for (int i = 0; i < 4; i++) {
		uint32_t channel = injector[i];
		__HAL_TIM_SET_OC_MODE(&htim1, channel, TIM_OCMODE_FORCED_INACTIVE);
		injector_state[i].state = INJ_STATE_IDLE;
	}
}

void ENGINE_Injector_OutputCompareCallback(uint32_t current_time) {
	/*
	if (injector_fire_loop_test) {
		uint32_t loop_check = ENGINE_CalculateDeltaT(current_time,
				last_injector_fire_loop_tick);
		if (loop_check >= 4000) {
			ENGINE_Injector_FireLoop();
		}
	}

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
	*/
}

static const uint32_t IGN_IT_FLAGS[4] = {TIM_IT_CC1, TIM_IT_CC2, TIM_IT_CC3, TIM_IT_CC4};
static const uint32_t IGN_CLR_FLAGS[4]= {TIM_FLAG_CC1, TIM_FLAG_CC2, TIM_FLAG_CC3, TIM_FLAG_CC4};

void ENGINE_Schedule_Spark(uint8_t cyl_id, uint32_t start_tick, uint16_t dwell_us, uint16_t spark_time_us) {
	if (dwell_us < 500)
		dwell_us = 500;
	if (dwell_us > 10000)
		dwell_us = 10000;

    uint32_t start_dwell_cnt = start_tick - dwell_us;

    uint32_t channel = ignition[cyl_id];

    ignition_state[cyl_id].dwell_us = dwell_us;
    ignition_state[cyl_id].spark_time_us = spark_time_us;
    ignition_state[cyl_id].spark_start_time = start_tick;

    ignition_state[cyl_id].state = IGN_STATE_DWELL;

    __HAL_TIM_SET_COMPARE(&htim4, channel, start_dwell_cnt);

    __HAL_TIM_SET_OC_MODE(&htim4, channel, TIM_OCMODE_ACTIVE);

    __HAL_TIM_CLEAR_FLAG(&htim4, IGN_CLR_FLAGS[cyl_id]);
    __HAL_TIM_ENABLE_IT(&htim4, IGN_IT_FLAGS[cyl_id]);
}

void ENGINE_Ignition_FireNow(uint8_t ign, uint16_t dwell_us, uint16_t spark_time_us) {
	if (dwell_us < 500)
		dwell_us = 500;
	if (dwell_us > 10000)
		dwell_us = 10000;

	uint32_t channel = ignition[ign];

	ignition_state[ign].dwell_us = dwell_us;
	ignition_state[ign].spark_time_us = spark_time_us;
	ignition_state[ign].spark_start_time = __HAL_TIM_GET_COUNTER(&htim4);

	uint32_t turn_off_dwell = ignition_state[ign].spark_start_time + ignition_state[ign].dwell_us;

	__HAL_TIM_SET_COMPARE(&htim4, channel, turn_off_dwell);
	__HAL_TIM_SET_OC_MODE(&htim4, channel, TIM_OCMODE_FORCED_ACTIVE);

	__HAL_TIM_SET_OC_MODE(&htim4, channel, TIM_OCMODE_INACTIVE);

    __HAL_TIM_CLEAR_FLAG(&htim4, IGN_IT_FLAGS[ign]);
    __HAL_TIM_ENABLE_IT(&htim4, IGN_CLR_FLAGS[ign]);
}

void ENGINE_Ignition_TestLoop(void) {
    if (!ignition_loop_test) {
        return;
    }

    uint32_t current_tick = HAL_GetTick();
    if ((current_tick - last_ignition_loop_tick) < 2000) {
        return;
    }

    last_ignition_loop_tick = current_tick;

    for (int i = 0; i < 4; i++) {
    }
}

void ENGINE_Ignition_StopAll(void) {
	for (int i = 0; i < 4; i++) {
		uint32_t channel = ignition[i];
		__HAL_TIM_SET_OC_MODE(&htim4, channel, TIM_OCMODE_FORCED_INACTIVE);
		ignition_state[i].state = IGN_STATE_IDLE;
	}
}

void ENGINE_Ignition_OutputCompareCallback(uint32_t current_time) {

}
