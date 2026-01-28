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
typedef struct {
    uint16_t pulse_width_us;
    bool isArmed;
    bool isActive;
    uint32_t pulse_start_time;
} Injector_State_t;

extern const uint32_t injector[4];
extern Injector_State_t injector_state[4];
extern TIM_HandleTypeDef htim1;

/* IGNITION */
extern const uint32_t ignition[4];
extern TIM_HandleTypeDef htim4;

/* VR */
typedef struct {
    uint32_t crakshaft_angle;
    uint32_t camshaft_angle;
} ENGINE_Shafts_t;

extern ENGINE_Shafts_t shafts;

void ENGINE_VR_OutputCompareCallback(uint32_t current_time, VR_Sensor_t ckp_sensor, VR_Sensor_t cmp_sensor);
void ENGINE_Injector_OutputCompareCallback(uint32_t current_time);
void ENGINE_Ignition_OutputCompareCallback(uint32_t current_time);

void ENGINE_Injector_FireNow(uint8_t inj, uint16_t pulse_us);
void ENGINE_Injector_Schedule(uint8_t inj, uint16_t pulse_us);
void ENGINE_Injector_Trigger(uint8_t inj);
void ENGINE_Injector_StopAll(void);

#endif /* INC_ENGINE_CONTROL_H_ */
