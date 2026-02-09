/*
 * engine_control.h
 *
 *  Created on: 28 de jan. de 2026
 *      Author: Matheus Markies
 */

#ifndef INC_ENGINE_CONTROL_H_
#define INC_ENGINE_CONTROL_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "MAX9924_driver.h"

#define _USE_MATH_DEFINES
#include "math.h"

#define MICROS_TO_SECONDS 0.000001f

typedef struct {
    uint32_t tim5;   // CKP (32-bit, 1µs)
    uint32_t tim4;   // Ignição       (16-bit, 1µs)
    uint32_t tim1;   // Injeção       (16-bit, 1µs)
} TimerSnapshot_t;

extern TimerSnapshot_t timer_snapshot;

/* INJECTOR */
typedef enum {
	INJ_STATE_IDLE = 0,
	INJ_STATE_SCHEDULED = 1,
	INJ_STATE_PENDING = 2,
	INJ_STATE_ACTIVE = 3,
} InjectorState_t;

typedef struct {
    uint16_t pulse_width_us;
    InjectorState_t state;

    uint16_t pulse_start_time;
} Injector_State_t;

extern Injector_State_t injector_state[4];
extern const uint32_t injector[4];
extern const uint32_t injector_active_channel[4];
extern TIM_HandleTypeDef htim1;

/* IGNITION */
typedef enum {
    IGN_STATE_IDLE = 0,
    IGN_STATE_PENDING,
    IGN_STATE_DWELL,
    IGN_STATE_FIRED
} IgnitionState_t;

typedef struct {
	uint32_t dwell_us;
	uint16_t dwell_start_time;

    uint16_t spark_time_us;
    uint16_t spark_start_time;

    IgnitionState_t state;
} Ignition_State_t;

extern Ignition_State_t ignition_state[4];
extern const uint32_t ignition[4];
extern const uint32_t ignition_active_channel[4];
extern TIM_HandleTypeDef htim4;

/* VR */
typedef enum {
    STROKE_POWER,
    STROKE_EXHAUST,
    STROKE_INTAKE,
    STROKE_COMPRESSION,
    STROKE_UNKNOWN
} PistonPhase_e;

typedef struct {
    PistonPhase_e current_phase;
} CylinderStatus_t;

typedef struct {
    float crankshaft_angular_velocity;
    float camshaft_angular_velocity;

    float crankshaft_angular_acc;
    float camshaft_angular_acc;

    CylinderStatus_t cyl_status[4];

    float crankshaft_angle; //global_angle
    float camshaft_angle;
} ENGINE_t;

extern ENGINE_t engine;

typedef struct {
    uint16_t dwell_us;
    uint16_t spark_time_us;

    float dwell_angle;
    float spark_angle;

    float injector_start_angle;
    float injector_end_angle;
} CylinderTelemetry_t;

typedef struct {
	CylinderTelemetry_t cyl_telemetry[4];
} Clycle_Telemetry_t;

extern Clycle_Telemetry_t telemetry;

/* TEST */
extern uint8_t injector_schedule_test;
extern uint8_t injector_loop_test;
extern uint8_t ignition_schedule_test;
extern uint8_t ignition_loop_test;

extern TIM_HandleTypeDef htim5;

#define ANGLE_CYCLE 720.0f

#define __HAL_TIM_SET_OC_MODE(__HANDLE__, __CHANNEL__, __MODE__) \
do { \
    if ((__CHANNEL__) == TIM_CHANNEL_1) { \
        MODIFY_REG((__HANDLE__)->Instance->CCMR1, TIM_CCMR1_OC1M, (__MODE__)); \
    } else if ((__CHANNEL__) == TIM_CHANNEL_2) { \
        MODIFY_REG((__HANDLE__)->Instance->CCMR1, TIM_CCMR1_OC2M, ((__MODE__) << 8U)); \
    } else if ((__CHANNEL__) == TIM_CHANNEL_3) { \
        MODIFY_REG((__HANDLE__)->Instance->CCMR2, TIM_CCMR2_OC3M, (__MODE__)); \
    } else if ((__CHANNEL__) == TIM_CHANNEL_4) { \
        MODIFY_REG((__HANDLE__)->Instance->CCMR2, TIM_CCMR2_OC4M, ((__MODE__) << 8U)); \
    } \
} while(0)

char* ENGINE_GetPhaseName(PistonPhase_e phase);
float ENGINE_GetNextPhaseAngle(void);
void ENGINE_UpdateCylinderPhases(float global_angle);
void ENGINE_PhaseOCCallback(void);

void ENGINE_ScheduleControl(VR_Sensor_t ckp_sensor);

void ENGINE_CKP_Callback(VR_Sensor_t ckp_sensor);

void ENGINE_Schedule_Injection(uint8_t cyl_id, uint16_t start_tick, uint16_t pulse_us);
void ENGINE_Schedule_Dwell(uint8_t cyl_id, uint32_t delta_to_start, uint32_t dwell_us);
void ENGINE_Schedule_Spark(uint8_t cyl_id, uint32_t delta_to_start, uint16_t spark_time_us);

void ENGINE_Injector_FireNow(uint8_t inj, uint16_t pulse_us);
void ENGINE_Injector_TestLoop(void);
void ENGINE_Injector_ScheduleTestLoop(void);
void ENGINE_Injector_StopAll(void);

void ENGINE_Ignition_FireNow(uint8_t ign, uint16_t dwell_us, uint16_t spark_time_us);
void ENGINE_Ignition_TestLoop(void);
void ENGINE_Ignition_ScheduleTestLoop(void);
void ENGINE_Ignition_StopAll(void);

void ENGINE_Injector_OutputCompareCallback(TIM_HandleTypeDef *htim);
void ENGINE_Ignition_OutputCompareCallback(TIM_HandleTypeDef *htim);


#endif /* INC_ENGINE_CONTROL_H_ */
