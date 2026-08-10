/*
 * MAP_sensor.h
 *
 *  Created on: 10 de ago. de 2026
 *      Author: Matheus Markies
 */

#ifndef INC_MAP_SENSOR_H_
#define INC_MAP_SENSOR_H_

#include "main.h"
#include <stdbool.h>

/* ==========================================================================
 *  ADC  -  DEVE bater com a configuracao do CubeMX (ADC1)
 *
 *  PC1 = ADC1_INP11. IN10 e IN11 precisam estar em Single-ended:
 *  com IN10 em Differential o PC1 vira ADC1_INN10 e este canal nao existe.
 * ========================================================================== */

#define MAP_ADC_RESOLUTION      12          /* 12, 14 ou 16 */

#if   (MAP_ADC_RESOLUTION == 16)
  #define MAP_ADC_MAX           65535.0f
#elif (MAP_ADC_RESOLUTION == 14)
  #define MAP_ADC_MAX           16383.0f
#elif (MAP_ADC_RESOLUTION == 12)
  #define MAP_ADC_MAX           4095.0f
#else
  #error "MAP_ADC_RESOLUTION invalida"
#endif

#define MAP_ADC_CHANNEL         ADC_CHANNEL_11          /* PC1 - Internal_MAP */
#define MAP_ADC_SAMPLETIME      ADC_SAMPLETIME_64CYCLES_5
#define MAP_VREF                3.3f

#define MAP_R1                  5600.0f
#define MAP_R2                  10000.0f
#define MAP_DIVIDER_FACTOR      (MAP_R2 / (MAP_R1 + MAP_R2))

/* ==========================================================================
 *  SENSOR  -  transferencia ratiometrica do datasheet
 *             V_out = V_s * (A * P[kPa] + B)
 *  Sensor da placa: MPXA4250AC6U (SOP, 20-250 kPa absoluto)
 *  Datasheet NXP MPX4250A/MPXA4250A rev 8.0:
 *      VOUT = VCC * (P * 0.004 - 0.04)
 *      VCC = 5.1 V tipico (4.85 a 5.35), sensibilidade 20 mV/kPa
 *  Pinout SOP: 1=DNC, 2=VCC, 3=GND, 4=VOUT, 5..8=DNC
 * ========================================================================== */

#define MAP_SENSOR_A            0.004000f
#define MAP_SENSOR_B           -0.040000f
#define MAP_SUPPLY_NOMINAL      5.1f

/* Faixa util (datasheet: 20 a 250 kPa; margem para o erro de calibracao) */
#define MAP_MIN_KPA             18.0f
#define MAP_MAX_KPA             255.0f

/* Deteccao de chicote: raw colado nos trilhos = fio aberto ou em curto */
#define MAP_RAW_OPEN            ((uint16_t)(MAP_ADC_MAX * 0.02f))
#define MAP_RAW_SHORT           ((uint16_t)(MAP_ADC_MAX * 0.98f))

#define MAP_FAULT_LIMIT         5           /* leituras ruins ate marcar falha */
#define MAP_CAL_SAMPLES         32          /* amostras por ponto de calibracao */
#define MAP_CYCLE_ALPHA         0.30f       /* EMA entre ciclos de 720 graus   */
#define MAP_DEFAULT_BARO        101.3f      /* fallback ao nivel do mar        */

typedef struct {
	float pressure;                 /* kPa */
	uint16_t raw_adc;
} MAP_Calibration_t;

typedef struct {
	/* leitura instantanea */
	float pressure;                 /* kPa da ultima conversao valida */
	uint16_t raw_adc;

	/* leitura sincronizada com o ciclo do motor */
	float pressure_cycle;           /* kPa, media filtrada do ultimo ciclo */
	float accumulator;
	uint16_t samples;

	/* barometrica aprendida com o motor parado */
	float baro;

	/* reta de conversao: kPa por LSB */
	float linear_cte;

	MAP_Calibration_t calL;
	MAP_Calibration_t calH;

	/* diagnostico */
	bool valid;
	uint16_t fault_count;
} MAP_t;

extern MAP_t map;

void MAP_Init(void);
void MAP_LoadDefaultCurve(void);

uint16_t MAP_ReadRaw(void);
bool MAP_Update(void);
float MAP_PressureFromRaw(uint16_t raw);

void MAP_OnCrankTooth(void);
void MAP_OnCycleComplete(void);

float MAP_GetPressure(void);            /* kPa do ciclo */
float MAP_GetInstant(void);             /* kPa instantaneo */
bool MAP_IsValid(void);
bool MAP_IsBusy(void);

/* --- calibracao --- */
bool MAP_LearnBaro(float known_kPa);    /* motor parado: desloca a reta        */
bool MAP_Calibrate_High(float known_kPa); /* segundo ponto: recalcula o ganho  */
void MAP_SetSupplyVoltage(float volts); /* compensacao ratiometrica            */

#endif /* INC_MAP_SENSOR_H_ */
