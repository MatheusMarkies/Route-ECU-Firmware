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

/* INJECTOR */
typedef enum {
	INJ_STATE_IDLE = 0,
	INJ_STATE_SCHEDULED = 1,
	INJ_STATE_ACTIVE = 2,
} InjectorState_t;

typedef struct {
    uint16_t pulse_width_us;
    InjectorState_t state;

    uint32_t pulse_start_time;
} Injector_State_t;

extern const uint32_t injector[4];
extern Injector_State_t injector_state[4];
extern TIM_HandleTypeDef htim1;

/* IGNITION */
typedef enum {
	IGN_STATE_IDLE = 0,
	IGN_STATE_DWELL = 1,
	IGN_STATE_SPARK = 2,
	IGN_STATE_RECOVERY = 3
} IgnitionState_t;

typedef struct {
    uint16_t dwell_us;
    uint16_t spark_time_us;
    uint32_t spark_start_time;

    IgnitionState_t state;
} Ignition_State_t;

extern Ignition_State_t ignition_state[4];
extern const uint32_t ignition[4];
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
    uint32_t crakshaft_angular_velocity;
    uint32_t camshaft_angular_velocity;

    CylinderStatus_t cyl_status[4];

    uint32_t crakshaft_angle;
    uint32_t camshaft_angle;
} ENGINE_t;

extern ENGINE_t engine;

extern uint8_t injector_loop_test;
extern uint8_t ignition_schedule_test;
extern uint8_t ignition_loop_test;

#define ANGLE_MAX 720

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

void ENGINE_CKP_Callback(VR_Sensor_t ckp_sensor);
void ENGINE_CMP_Callback(VR_Sensor_t ckp_sensor);

void ENGINE_VR_OutputCompareCallback(uint32_t current_time, VR_Sensor_t ckp_sensor, VR_Sensor_t cmp_sensor);
void ENGINE_Injector_OutputCompareCallback(uint32_t current_time);
void ENGINE_Ignition_OutputCompareCallback(uint32_t current_time);

void ENGINE_Schedule_Injection(uint8_t cyl_id, uint16_t start_tick, uint16_t pulse_us);
void ENGINE_Injector_FireNow(uint8_t inj, uint16_t pulse_us);
void ENGINE_Injector_TestLoop(void);
void ENGINE_Injector_StopAll(void);

void ENGINE_Schedule_Spark(uint8_t cyl_id, uint32_t start_tick, uint16_t dwell_us, uint16_t spark_time_us) ;
void ENGINE_Ignition_FireNow(uint8_t ign, uint16_t dwell_us, uint16_t spark_time_us) ;
void ENGINE_Ignition_TestLoop(void);
void ENGINE_Ignition_ScheduleTestLoop(void);
void ENGINE_Ignition_StopAll(void);

#endif /* INC_ENGINE_CONTROL_H_ */
