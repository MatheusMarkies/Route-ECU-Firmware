/*
 * crank_decoder.h
 *
 *  Created on: Aug 20, 2025
 *      Author: Matheus Markies
 */

#ifndef INC_CRANK_DECODER_H_
#define INC_CRANK_DECODER_H_

#include "main.h"
#include "ecu_config.h"
#include "engine_control.h"
#include <math.h>

extern const long TIMER_CLOCK_FREQ_HZ;

// --- Variáveis Externas ---
extern TIM_HandleTypeDef htim2;

// --- Definição das Variáveis Globais ---
extern EcuConfig_t  g_ecuConfig;
extern EngineData_t g_engineData;
extern EngineState_t g_engineState;

// --- Variáveis Privadas ---
static volatile uint32_t last_capture_tick = 0;
static volatile uint32_t last_dt_ticks = 0;
static volatile int engine_is_synced = 0;
static volatile uint16_t tooth_counter = 0;

void Crank_Init(void);

#endif /* INC_CRANK_DECODER_H_ */
