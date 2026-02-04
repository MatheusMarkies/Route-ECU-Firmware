/*
 * engine_control.c
 *
 *  Created on: 28 de jan. de 2026
 *      Author: Matheus Markies
 */

#include "engine_control.h"
#include "stdio.h"
#include "string.h"

uint8_t injector_schedule_test = 0;
static uint32_t last_inj_schedule_test_tick = 0;

uint8_t injector_loop_test = 0;
uint32_t last_injector_loop_tick;

uint8_t ignition_schedule_test = 0;
uint32_t last_schedule_test_tick = 0;

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

static uint32_t ENGINE_CalculateDeltaTBasedOnAngle(float current_angle,
		float target_angle, float angular_velocity_deg_s) {
	float delta_angle;

	if (angular_velocity_deg_s <= 0.1f)
		return 0;

	if (target_angle >= current_angle) {
		delta_angle = target_angle - current_angle;
	} else {
		delta_angle = (ANGLE_MAX - current_angle) + target_angle;
	}

	return (uint32_t) ((delta_angle / angular_velocity_deg_s) * 1000000.0f);
}

static float ENGINE_CalculateAngularVelocity(uint32_t current,
		VR_Sensor_t sensor) {
	if (sensor.frequency_hz <= 0.0f || !sensor.isSync)
		return 0.0f;
	return 360.0f * sensor.frequency_hz; //Graus/seg
}

float ENGINE_CalculateAngle(uint32_t current, VR_Sensor_t sensor) {
	if (sensor.frequency_hz <= 0.0f || !sensor.isSync)
		return 0.0f;

	float degrees_per_tooth = 360.0f / (float) sensor.tooths;
	float fixed_angle = degrees_per_tooth * (float) sensor.pulse_count;

	float angular_velocity = ENGINE_CalculateAngularVelocity(current, sensor);
	float dt_seconds = (float) ENGINE_CalculateDeltaT(current,
			sensor.current_edge_time) * MICROS_TO_SECONDS;

	float final_angle = fixed_angle + (angular_velocity * dt_seconds);

	while (final_angle >= ANGLE_MAX)
		final_angle -= ANGLE_MAX;
	while (final_angle < 0.0f)
		final_angle += ANGLE_MAX;

	return final_angle;
}

const char* ENGINE_GetPhaseName(PistonPhase_e phase) {
    switch (phase) {
        case STROKE_POWER:       return "POWER";       // Expansão
        case STROKE_EXHAUST:     return "EXHAUST";     // Escape
        case STROKE_INTAKE:      return "INTAKE";      // Admissão
        case STROKE_COMPRESSION: return "COMPRESSION"; // Compressão
        default:                 return "UNKNOWN";
    }
}

static const float FIRING_OFFSET[4] = { 0.0f,   // Cyl 1 (0 grau)
		540.0f, // Cyl 2 (540 graus)
		180.0f, // Cyl 3 (180 graus)
		360.0f  // Cyl 4 (360 graus)
		};

void ENGINE_UpdateCylinderPhases(float global_angle) {
	for (int i = 0; i < 4; i++) {
		float local_angle = global_angle - FIRING_OFFSET[i];

		while (local_angle < 0.0f)
			local_angle += 720.0f;
		while (local_angle >= 720.0f)
			local_angle -= 720.0f;

		if (local_angle < 180.0f) {
			engine.cyl_status[i].current_phase = STROKE_POWER; // Descendo (Explosão)
		} else if (local_angle < 360.0f) {
			engine.cyl_status[i].current_phase = STROKE_EXHAUST; // Subindo (Escape)
		} else if (local_angle < 540.0f) {
			engine.cyl_status[i].current_phase = STROKE_INTAKE; // Descendo (Admissão)
		} else {
			engine.cyl_status[i].current_phase = STROKE_COMPRESSION; // Subindo (Compressão)
		}
	}
}

void ENGINE_ScheduleNextPhase(uint32_t current_time_base) {
	float current_angle = (float) engine.crakshaft_angle;
	float next_target = 0.0f;

	if (current_angle < 180.0f)
		next_target = 180.0f;
	else if (current_angle < 360.0f)
		next_target = 360.0f;
	else if (current_angle < 540.0f)
		next_target = 540.0f;
	else
		next_target = 720.0f;

	uint32_t time_to_target_us = ENGINE_CalculateDeltaTBasedOnAngle(
			current_angle, (next_target >= 720.0f) ? 0.0f : next_target,
			(float) engine.crakshaft_angular_velocity);

	uint32_t match_value = current_time_base + time_to_target_us;

	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, match_value);

	__HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC4);
	__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC4);
}

void ENGINE_CKP_Callback(VR_Sensor_t ckp_sensor) {
	uint32_t current_inj_time = __HAL_TIM_GET_COUNTER(&htim1);
	uint32_t current_ign_time = __HAL_TIM_GET_COUNTER(&htim4);

	uint32_t current_ckp_time = __HAL_TIM_GET_COUNTER(&htim5);

	engine.crakshaft_angular_velocity = ENGINE_CalculateAngularVelocity(
			current_ckp_time, ckp_sensor);
	engine.crakshaft_angle = ENGINE_CalculateAngle(current_ckp_time,
			ckp_sensor); //<- Zera o angulo ja que estamos no 0

	if (ckp_sensor.isSync) {
		ENGINE_ScheduleNextPhase(current_ckp_time);
	}
}

void ENGINE_CMP_Callback(VR_Sensor_t cmp_sensor) {
	uint32_t current_inj_time = __HAL_TIM_GET_COUNTER(&htim1);
	uint32_t current_ign_time = __HAL_TIM_GET_COUNTER(&htim4);

	uint32_t current_cmp_time = __HAL_TIM_GET_COUNTER(&htim5);

	engine.camshaft_angular_velocity = ENGINE_CalculateAngularVelocity(
			current_cmp_time, cmp_sensor);
	engine.camshaft_angle = ENGINE_CalculateAngle(current_cmp_time, cmp_sensor); //<- Zera o angulo ja que estamos no 0
}

void ENGINE_VR_OutputCompareCallback(uint32_t current_time,
		VR_Sensor_t ckp_sensor, VR_Sensor_t cmp_sensor) {
	engine.crakshaft_angular_velocity = ENGINE_CalculateAngularVelocity(
			current_time, ckp_sensor);
	engine.camshaft_angular_velocity = ENGINE_CalculateAngularVelocity(
			current_time, cmp_sensor);

	engine.crakshaft_angle = ENGINE_CalculateAngle(current_time, ckp_sensor);
	engine.camshaft_angle = ENGINE_CalculateAngle(current_time, cmp_sensor);
}

static const uint32_t INJ_IT_FLAGS[4] = { TIM_IT_CC1, TIM_IT_CC2, TIM_IT_CC3,
TIM_IT_CC4 };
static const uint32_t INJ_CLR_FLAGS[4] = { TIM_FLAG_CC1, TIM_FLAG_CC2,
TIM_FLAG_CC3, TIM_FLAG_CC4 };

void ENGINE_Schedule_Injection(uint8_t cyl_id, uint16_t start_tick,
		uint16_t pulse_us) {

	uint32_t channel = injector[cyl_id];

	injector_state[cyl_id].pulse_width_us = pulse_us;
	injector_state[cyl_id].pulse_start_time = start_tick;

	injector_state[cyl_id].state = INJ_STATE_SCHEDULED;

	__HAL_TIM_SET_COMPARE(&htim1, channel,
			injector_state[cyl_id].pulse_start_time);

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

	uint32_t turn_off_cnt = injector_state[inj].pulse_start_time
			+ injector_state[inj].pulse_width_us;

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

	printf("[INJ TEST] Inject! \r\n");

	for (int i = 0; i < 4; i++) {
		ENGINE_Injector_FireNow(i, (uint16_t) 300000);
	}
}

void ENGINE_Injector_ScheduleTestLoop(void) {
	if (!injector_schedule_test) {
		return;
	}

	uint32_t current_tick = HAL_GetTick();

	if ((current_tick - last_inj_schedule_test_tick) < 500) {
		return;
	}

	if (injector_state[0].state != INJ_STATE_IDLE) {
		return;
	}

	last_inj_schedule_test_tick = current_tick;

	uint32_t current_timer_cnt = __HAL_TIM_GET_COUNTER(&htim1);
	uint32_t delay_into_future_us = 200000; //20ms
	uint16_t target_start_tick = (uint16_t) (current_timer_cnt
			+ delay_into_future_us);

	printf("[INJ TEST] Scheduling sequence initiated...\r\n");

	for (int i = 0; i < 4; i++) {
		ENGINE_Schedule_Injection(i, target_start_tick, 50000);
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

}

static const uint32_t IGN_IT_FLAGS[4] = { TIM_IT_CC1, TIM_IT_CC2, TIM_IT_CC3,
TIM_IT_CC4 };
static const uint32_t IGN_CLR_FLAGS[4] = { TIM_FLAG_CC1, TIM_FLAG_CC2,
TIM_FLAG_CC3, TIM_FLAG_CC4 };

void ENGINE_Schedule_Spark(uint8_t cyl_id, uint32_t start_tick,
		uint16_t dwell_us, uint16_t spark_time_us) {
	if (dwell_us < 2000)
		dwell_us = 2000;
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

void ENGINE_Ignition_FireNow(uint8_t ign, uint16_t dwell_us,
		uint16_t spark_time_us) {
	if (dwell_us < 2000)
		dwell_us = 2000;
	if (dwell_us > 10000)
		dwell_us = 10000;

	uint32_t channel = ignition[ign];

	ignition_state[ign].dwell_us = dwell_us;
	ignition_state[ign].spark_time_us = spark_time_us;
	ignition_state[ign].spark_start_time = __HAL_TIM_GET_COUNTER(&htim4);

	uint32_t turn_off_dwell = ignition_state[ign].spark_start_time
			+ ignition_state[ign].dwell_us;

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
	if ((current_tick - last_ignition_loop_tick) < 1000) { //ms
		return;
	}

	last_ignition_loop_tick = current_tick;

	printf("[IGN TEST] Ignition! \r\n");

	for (int i = 0; i < 4; i++) {
		ENGINE_Ignition_FireNow(i, 4000, 0); //us
	}
}

void ENGINE_Ignition_ScheduleTestLoop(void) {
	if (!ignition_schedule_test) {
		return;
	}

	uint32_t current_tick = HAL_GetTick();

	if ((current_tick - last_schedule_test_tick) < 500) {
		return;
	}

	if (ignition_state[0].state != IGN_STATE_IDLE) {
		return;
	}

	last_schedule_test_tick = current_tick;

	printf("[IGN TEST] Scheduling sequence initiated...\r\n");

	uint32_t current_timer_cnt = __HAL_TIM_GET_COUNTER(&htim4);
	uint32_t delay_into_future_us = 20000;

	uint32_t target_spark_tick = current_timer_cnt + delay_into_future_us;

	for (int i = 0; i < 4; i++) {
		ENGINE_Schedule_Spark(i, target_spark_tick, 4000, 0);
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
