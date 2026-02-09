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
static uint32_t last_injector_loop_tick = 0;

uint8_t ignition_schedule_test = 0;
static uint32_t last_schedule_test_tick = 0;

uint8_t ignition_loop_test = 0;
static uint32_t last_ignition_loop_tick = 0;

Injector_State_t injector_state[4] = { 0 };
Ignition_State_t ignition_state[4] = { 0 };

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

const uint32_t injector_active_channel[4] = { HAL_TIM_ACTIVE_CHANNEL_1,
		HAL_TIM_ACTIVE_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_3,
		HAL_TIM_ACTIVE_CHANNEL_4 };

const uint32_t ignition_active_channel[4] = { HAL_TIM_ACTIVE_CHANNEL_1,
		HAL_TIM_ACTIVE_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_3,
		HAL_TIM_ACTIVE_CHANNEL_4 };

static const uint32_t IT_FLAGS[4] = { TIM_IT_CC1, TIM_IT_CC2, TIM_IT_CC3,
TIM_IT_CC4 };
static const uint32_t CLR_FLAGS[4] = { TIM_FLAG_CC1, TIM_FLAG_CC2,
TIM_FLAG_CC3, TIM_FLAG_CC4 };

ENGINE_t engine = { 0 };
Clycle_Telemetry_t telemetry = { 0 };
TimerSnapshot_t timer_snapshot = { 0 };

static const float FIRING_OFFSET[4] = {
		0.0f,   // Cyl 1 (0 graus)
		540.0f, // Cyl 2 (540 graus)
		180.0f, // Cyl 3 (180 graus)
		360.0f  // Cyl 4 (360 graus)
};

uint32_t MIN_GUARD_TIME_US = 50;       // Tempo mínimo para agendar OC
uint32_t MAX_SCHEDULE_TIME_US = 5000;    // 5ms — clamp para evitar overflow em baixas RPM

static int firing_cylinder = 0;
static int injection_cylinder = 0;

static inline float wrap(float angle, float max) {
    while (angle >= max) angle -= max;
    while (angle < 0.0f) angle += max;
    return angle;
}

static inline float clampf(float val, float min, float max) {
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    }
    return val;
}

static void ENGINE_CaptureTimers(void) {
	timer_snapshot.tim5 = __HAL_TIM_GET_COUNTER(&htim5);
	timer_snapshot.tim4 = __HAL_TIM_GET_COUNTER(&htim4);
	timer_snapshot.tim1 = __HAL_TIM_GET_COUNTER(&htim1);
}

char* ENGINE_GetPhaseName(PistonPhase_e phase) {
	switch (phase) {
	case STROKE_POWER:
		return "POWER";       // Expansão
	case STROKE_EXHAUST:
		return "EXHAUST";     // Escape
	case STROKE_INTAKE:
		return "INTAKE";      // Admissão
	case STROKE_COMPRESSION:
		return "COMPRESSION"; // Compressão
	default:
		return "UNKNOWN";
	}
}

static uint32_t ENGINE_CalculateDeltaT(uint32_t current, uint32_t previous) {
	if (current >= previous) {
		return current - previous;
	} else {
		// Overflow do timer
		return (0xFFFFFFFF - previous) + current + 1;
	}
}

static inline uint32_t ENGINE_Tim5TickAfter(uint32_t now_tim5, uint32_t delay_us) {
	return (now_tim5 + delay_us) & 0xFFFFFFFF;  // TIM5 = 32 bits
}

static inline uint32_t ENGINE_Tim4TickAfter(uint32_t now_tim4, uint32_t delay_us) {
    return (now_tim4 + delay_us) & 0xFFFF;  // TIM4 = 16 bits
}

static inline uint32_t ENGINE_Tim1TickAfter(uint32_t now_tim1, uint32_t delay_us) {
    return (now_tim1 + delay_us) & 0xFFFF;  // TIM1 = 16 bits
}

static float ENGINE_DeltaAngle(float from, float to) {
    float delta = to - from;
    if (delta < 0.0f)
        delta += ANGLE_CYCLE;
    return delta;
}

static float ENGINE_CalculateAngularVelocity(VR_Sensor_t sensor) {

	if (!sensor.isSync)
		return 0.0f;

	if (sensor.frequency_hz <= 0.0f || sensor.period == 0 || sensor.tooths == 0)
		return 0.0f;

	float delta_theta = 360.0f / (float) sensor.tooths;
	float omega_now = delta_theta / ((float) sensor.period * 1e-6f);

	return omega_now; //Graus/seg
}

static float ENGINE_CalculateAngularAcceleration(VR_Sensor_t sensor) {
	if (!sensor.isSync)
		return 0.0f;

	if (sensor.frequency_hz <= 0.0f || sensor.period == 0 || sensor.last_period == 0)
		return 0.0f;

	float delta_theta = 360.0f / sensor.tooths;

	float omega_now = delta_theta / (sensor.period * 1e-6f);
	float omega_prev = delta_theta / (sensor.last_period * 1e-6f);

	float delta_t_avg = 0.5f * ((sensor.period + sensor.last_period) * 1e-6f);

	float angular_acc = (omega_now - omega_prev) / delta_t_avg;

	return angular_acc; //Graus/seg^2
}


static float ENGINE_CalculateFixedAngle(VR_Sensor_t sensor) {
	if (!sensor.isSync)
		return 0.0f;

	if (sensor.frequency_hz <= 0.0f || sensor.period == 0 || sensor.tooths == 0)
		return 0.0f;

	float degrees_per_tooth = 360.0f / (float) sensor.tooths;
	float fixed_angle = degrees_per_tooth * (float) sensor.pulse_count;

	return wrap(fixed_angle, ANGLE_CYCLE);
}

static float ENGINE_CalculateAngle(VR_Sensor_t sensor) {
	if (!sensor.isSync)
		return 0.0f;

	if (sensor.frequency_hz <= 0.0f || sensor.period == 0 || sensor.tooths == 0)
		return 0.0f;

	float degrees_per_tooth = 360.0f / sensor.tooths;

	float base_angle = degrees_per_tooth * sensor.pulse_count;
	float dt = (float) ENGINE_CalculateDeltaT(timer_snapshot.tim5, sensor.current_edge_time); // us

	float frac = dt / (float) sensor.period;

	float angle = base_angle + frac * degrees_per_tooth;

	return wrap(angle, ANGLE_CYCLE);
}

float ENGINE_GetNextPhaseAngle(void){
	float best = 0.0f;
	float lowerDelta = ANGLE_CYCLE;

	for(int i = 0;i < 4; i++){
		float nextDelta = ENGINE_DeltaAngle(engine.crankshaft_angle, FIRING_OFFSET[i]);

		if(nextDelta < lowerDelta){
			lowerDelta = nextDelta;
			best = FIRING_OFFSET[i];
		}
	}

	return best;
}

void ENGINE_UpdateCylinderPhases(float global_angle) {
	for (int i = 0; i < 4; i++) {
		float local_angle = global_angle - FIRING_OFFSET[i];

		local_angle = wrap(local_angle, ANGLE_CYCLE);

		if (local_angle >= 0.0f && local_angle < 180.0f) {
			engine.cyl_status[i].current_phase = STROKE_POWER; // Descendo (Explosão)
		} else if (local_angle >= 180.0f && local_angle < 360.0f) {
			engine.cyl_status[i].current_phase = STROKE_EXHAUST; // Subindo (Escape)
		} else if (local_angle >= 360.0f && local_angle < 540.0f) {
			engine.cyl_status[i].current_phase = STROKE_INTAKE; // Descendo (Admissão)
		} else if (local_angle >= 540.0f && local_angle < 720.0f) {
			engine.cyl_status[i].current_phase = STROKE_COMPRESSION; // Subindo (Compressão)
		}
	}
}

void ENGINE_PhaseOCCallback(void){
	ENGINE_CaptureTimers();

	engine.crankshaft_angle = ENGINE_CalculateAngle(ckp_sensor);
	engine.crankshaft_angular_velocity = ENGINE_CalculateAngularVelocity(ckp_sensor);
	engine.crankshaft_angular_acc = ENGINE_CalculateAngularAcceleration(ckp_sensor);

	ENGINE_UpdateCylinderPhases(engine.crankshaft_angle);
}

void ENGINE_ScheduleControl(VR_Sensor_t ckp_sensor){
	ENGINE_CaptureTimers();

	engine.crankshaft_angle = ENGINE_CalculateAngle(ckp_sensor);
	engine.crankshaft_angular_velocity = ENGINE_CalculateAngularVelocity(ckp_sensor);
	engine.crankshaft_angular_acc = ENGINE_CalculateAngularAcceleration(ckp_sensor);

	float next_phase_angle = ENGINE_GetNextPhaseAngle();
	engine.camshaft_angle = next_phase_angle;

	float delta_to_phase_angle = ENGINE_DeltaAngle(engine.crankshaft_angle, next_phase_angle);

	float time_to_next_phase_s = delta_to_phase_angle / engine.crankshaft_angular_velocity;
	uint32_t time_to_next_phase_us = (uint32_t)(time_to_next_phase_s * 1e6);

	if (time_to_next_phase_us < MIN_GUARD_TIME_US)
		return;

	time_to_next_phase_us = clampf(time_to_next_phase_us, MIN_GUARD_TIME_US, MAX_SCHEDULE_TIME_US);
	uint32_t match = ENGINE_Tim5TickAfter(timer_snapshot.tim5, time_to_next_phase_us);

	for(int i = 0; i<4;i++){
		if(FIRING_OFFSET[i] == next_phase_angle){
			firing_cylinder = i;
			break;
		}
	}

	for(int i = 0; i<4;i++){
		if(wrap(FIRING_OFFSET[i] + 360.0f, ANGLE_CYCLE) == next_phase_angle){
			injection_cylinder = i;
			break;
		}
	}

	uint32_t dwell_us = 2000;
	float dwell_angle = engine.crankshaft_angular_velocity * ((float)dwell_us * 1e-6);

	float delta_to_dwell_angle = ENGINE_DeltaAngle(dwell_angle, next_phase_angle);

	float time_to_dwell_s = delta_to_dwell_angle / engine.crankshaft_angular_velocity;
	uint32_t time_to_dwell_us = (uint32_t)(time_to_dwell_s * 1e6);

	float tooths_to_dwell = (float)dwell_us / ckp_sensor.period;
	tooths_to_dwell = clampf(tooths_to_dwell, 1.0f, (float)ckp_sensor.tooths);
	if(delta_to_phase_angle < (360.0f/(float)ckp_sensor.tooths * tooths_to_dwell + 1.0f)){
		ENGINE_Schedule_Dwell(firing_cylinder, time_to_dwell_us, dwell_us);
	}

	if(delta_to_phase_angle < (360.0f/(float)ckp_sensor.tooths + 1.0f)){
		ENGINE_Schedule_Injection(injection_cylinder, time_to_next_phase_us, 20000);
	}

	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, match);
	__HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC4);
	__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC4);
}

static uint8_t first_is_sync = 0;
void ENGINE_CKP_Callback(VR_Sensor_t ckp_sensor) {
	if (!ckp_sensor.isSync)
		return;

	if (ckp_sensor.frequency_hz <= 0.0f || ckp_sensor.period == 0 || ckp_sensor.tooths == 0)
		return;

    if(!first_is_sync)
    	ENGINE_PhaseOCCallback();

    first_is_sync = 1;
    ENGINE_ScheduleControl(ckp_sensor);
}

void ENGINE_Schedule_Injection(uint8_t cyl_id, uint16_t delta_to_start, uint16_t pulse_us) {
	ENGINE_CaptureTimers();

	uint32_t channel = injector[cyl_id];

	uint16_t match = ENGINE_Tim1TickAfter(timer_snapshot.tim1, delta_to_start);

	injector_state[cyl_id].pulse_width_us = pulse_us;
	injector_state[cyl_id].pulse_start_time = match;

	injector_state[cyl_id].state = INJ_STATE_SCHEDULED;

	__HAL_TIM_SET_OC_MODE(&htim1, channel, TIM_OCMODE_ACTIVE);
	__HAL_TIM_SET_COMPARE(&htim1, channel, match);

	__HAL_TIM_CLEAR_FLAG(&htim1, CLR_FLAGS[cyl_id]);
	__HAL_TIM_ENABLE_IT(&htim1, IT_FLAGS[cyl_id]);
}

void ENGINE_Injector_FireNow(uint8_t inj, uint16_t pulse_us) {
	ENGINE_CaptureTimers();

    uint32_t channel = injector[inj];

    injector_state[inj].pulse_width_us = pulse_us;
    injector_state[inj].pulse_start_time = timer_snapshot.tim1;
    injector_state[inj].state = INJ_STATE_ACTIVE;

    __HAL_TIM_SET_OC_MODE(&htim1, channel, TIM_OCMODE_FORCED_ACTIVE);

    uint16_t match = ENGINE_Tim1TickAfter(timer_snapshot.tim1, pulse_us);

    injector_state[inj].state = INJ_STATE_ACTIVE;

	__HAL_TIM_SET_OC_MODE(&htim1, channel, TIM_OCMODE_INACTIVE);
	__HAL_TIM_SET_COMPARE(&htim1, channel, match);

	__HAL_TIM_CLEAR_FLAG(&htim1, CLR_FLAGS[inj]);
	__HAL_TIM_ENABLE_IT(&htim1, IT_FLAGS[inj]);
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
	uint32_t delay_into_future_us = 20000; //20ms
	uint16_t target_start_tick = (uint16_t) (current_timer_cnt
			+ delay_into_future_us);

	printf("[INJ TEST] Scheduling sequence initiated...\r\n");

	for (int i = 0; i < 4; i++) {
		ENGINE_Schedule_Injection(i, target_start_tick, (uint16_t)50000);
	}
}

void ENGINE_Injector_StopAll(void) {
	for (int i = 0; i < 4; i++) {
		uint32_t channel = injector[i];
		__HAL_TIM_SET_OC_MODE(&htim1, channel, TIM_OCMODE_FORCED_INACTIVE);
		injector_state[i].state = INJ_STATE_IDLE;
	}
}

void ENGINE_Injector_OutputCompareCallback(TIM_HandleTypeDef *htim) {
    for (int i = 0; i < 4; i++) {
        if (htim->Channel != injector_active_channel[i])
            continue;

		uint32_t channel = injector[i];
		switch (injector_state[i].state) {

		case INJ_STATE_SCHEDULED:
			ENGINE_CaptureTimers();

			injector_state[i].state = INJ_STATE_ACTIVE;

			uint16_t match = ENGINE_Tim1TickAfter(timer_snapshot.tim1, injector_state[i].pulse_width_us);

			__HAL_TIM_SET_OC_MODE(&htim1, channel, TIM_OCMODE_INACTIVE);
			__HAL_TIM_SET_COMPARE(&htim1, channel, match);

			__HAL_TIM_CLEAR_FLAG(&htim1, CLR_FLAGS[i]);
			__HAL_TIM_ENABLE_IT(&htim1, IT_FLAGS[i]);
			break;

		case INJ_STATE_ACTIVE:
			__HAL_TIM_DISABLE_IT(&htim1, IT_FLAGS[i]);
			injector_state[i].state = INJ_STATE_IDLE;
			break;

		default:
			__HAL_TIM_DISABLE_IT(&htim1, IT_FLAGS[i]);
			injector_state[i].state = INJ_STATE_IDLE;
			break;
		}
    }
}

void ENGINE_Schedule_Dwell(uint8_t cyl_id, uint32_t delta_to_start, uint32_t dwell_us) {
	ENGINE_CaptureTimers();

	uint32_t channel = ignition[cyl_id];

	uint16_t start_tick = ENGINE_Tim4TickAfter(timer_snapshot.tim4, delta_to_start);

	ignition_state[cyl_id].dwell_us = dwell_us;
	ignition_state[cyl_id].dwell_start_time = start_tick;

	ignition_state[cyl_id].state = IGN_STATE_DWELL;

	__HAL_TIM_SET_OC_MODE(&htim4, channel, TIM_OCMODE_ACTIVE);
	__HAL_TIM_SET_COMPARE(&htim4, channel, start_tick);

	__HAL_TIM_CLEAR_FLAG(&htim4, CLR_FLAGS[cyl_id]);
	__HAL_TIM_ENABLE_IT(&htim4, IT_FLAGS[cyl_id]);
}

void ENGINE_Schedule_Spark(uint8_t cyl_id, uint32_t delta_to_start, uint16_t spark_time_us) {
	ENGINE_CaptureTimers();

	uint32_t channel = ignition[cyl_id];

	uint16_t start_tick = ENGINE_Tim4TickAfter(timer_snapshot.tim4, delta_to_start);

	ignition_state[cyl_id].spark_time_us = spark_time_us;
	ignition_state[cyl_id].spark_start_time = start_tick;

	__HAL_TIM_SET_OC_MODE(&htim4, channel, TIM_OCMODE_INACTIVE);
	__HAL_TIM_SET_COMPARE(&htim4, channel, start_tick);

	__HAL_TIM_CLEAR_FLAG(&htim4, CLR_FLAGS[cyl_id]);
	__HAL_TIM_ENABLE_IT(&htim4, IT_FLAGS[cyl_id]);
}

void ENGINE_Ignition_FireNow(uint8_t ign, uint16_t dwell_us, uint16_t spark_time_us) {
	ENGINE_CaptureTimers();

	uint32_t channel = ignition[ign];

	ignition_state[ign].dwell_us = dwell_us;
	ignition_state[ign].spark_time_us = spark_time_us;
	ignition_state[ign].spark_start_time = timer_snapshot.tim4;

	ignition_state[ign].state = IGN_STATE_DWELL;

	__HAL_TIM_SET_OC_MODE(&htim4, channel, TIM_OCMODE_FORCED_ACTIVE);

	uint16_t turn_off_dwell = ENGINE_Tim4TickAfter(ignition_state[ign].spark_start_time, ignition_state[ign].dwell_us);

	__HAL_TIM_SET_OC_MODE(&htim4, channel, TIM_OCMODE_INACTIVE);
	__HAL_TIM_SET_COMPARE(&htim4, channel, turn_off_dwell);

	__HAL_TIM_CLEAR_FLAG(&htim4, CLR_FLAGS[ign]);
	__HAL_TIM_ENABLE_IT(&htim4, IT_FLAGS[ign]);
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

	uint16_t delay_into_future_us = 20000;

	for (int i = 0; i < 4; i++) {
		ENGINE_Schedule_Spark(i, delay_into_future_us, 0);
	}
}

void ENGINE_Ignition_StopAll(void) {
	for (int i = 0; i < 4; i++) {
		uint32_t channel = ignition[i];
		__HAL_TIM_SET_OC_MODE(&htim4, channel, TIM_OCMODE_FORCED_INACTIVE);
		ignition_state[i].state = IGN_STATE_IDLE;
	}
}

void ENGINE_Ignition_OutputCompareCallback(TIM_HandleTypeDef *htim) {
    for (int i = 0; i < 4; i++) {
        if (htim->Channel != ignition_active_channel[i])
            continue;

        uint32_t channel = ignition[i];

        switch (ignition_state[i].state) {
        case IGN_STATE_PENDING: {

            break;
        }
        case IGN_STATE_DWELL:
        	ENGINE_CaptureTimers();

        	uint32_t spark_time = ENGINE_Tim4TickAfter(timer_snapshot.tim4, ignition_state[i].dwell_us);

        	__HAL_TIM_SET_OC_MODE(htim, channel, TIM_OCMODE_INACTIVE);
        	__HAL_TIM_SET_COMPARE(htim, channel, spark_time);

        	ignition_state[i].state = IGN_STATE_FIRED;
            break;
        case IGN_STATE_FIRED:
        	__HAL_TIM_DISABLE_IT(htim, IT_FLAGS[i]);
        	ignition_state[i].state = IGN_STATE_IDLE;
        	__HAL_TIM_SET_OC_MODE(htim, channel, TIM_OCMODE_FORCED_INACTIVE);
            break;
        default:
        	__HAL_TIM_DISABLE_IT(htim, IT_FLAGS[i]);
            break;
        }

        return;
    }
}
