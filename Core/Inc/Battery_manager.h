/*
 * Battery_manager.h
 *
 *  Created on: 28 de jan. de 2026
 *  Revisado em: 10 de ago. de 2026
 *      Author: Matheus Markies
 */

#ifndef INC_BATTERY_MANAGER_H_
#define INC_BATTERY_MANAGER_H_

#include "main.h"
#include <stdbool.h>

#define BATTERY_ADC_RESOLUTION      12          /* 12, 14 ou 16 */

#if   (BATTERY_ADC_RESOLUTION == 16)
  #define BATTERY_ADC_MAX           65535.0f
#elif (BATTERY_ADC_RESOLUTION == 14)
  #define BATTERY_ADC_MAX           16383.0f
#elif (BATTERY_ADC_RESOLUTION == 12)
  #define BATTERY_ADC_MAX           4095.0f
#else
  #error "BATTERY_ADC_RESOLUTION invalida"
#endif

#define BATTERY_ADC_CHANNEL         ADC_CHANNEL_10          /* PC0 - Bat_Voltage */
#define BATTERY_ADC_SAMPLETIME      ADC_SAMPLETIME_64CYCLES_5
#define BATTERY_VREF                3.3f

/* Divisor: V_bat -> R1 (47k) -> no do ADC -> R2 (10k) -> GND
 * Fator 0,17544, fundo de escala 18,81 V.
 * Impedancia Thevenin 8,25 k -> 64,5 ciclos de sampling sao o minimo seguro. */
#define BATTERY_R1                  47000.0f
#define BATTERY_R2                  10000.0f
#define BATTERY_DIVIDER_FACTOR      (BATTERY_R2 / (BATTERY_R1 + BATTERY_R2))
#define BATTERY_FULL_SCALE_V        (BATTERY_VREF / BATTERY_DIVIDER_FACTOR)

#define BATTERY_MIN_VOLTAGE         4.0f
#define BATTERY_MAX_VOLTAGE         18.35f

#define BATTERY_RAW_OPEN            ((uint16_t)(BATTERY_ADC_MAX * 0.02f))
#define BATTERY_RAW_SHORT           ((uint16_t)(BATTERY_ADC_MAX * 0.98f))

#define BATTERY_DEFAULT_VOLTAGE     13.8f

#define BATTERY_FAULT_LIMIT         5       /* leituras ruins ate marcar falha  */
#define BATTERY_PENDING_TIMEOUT     8       /* chamadas ate abortar conversao   */
#define BATTERY_CAL_SAMPLES         32      /* amostras por ponto de calibracao */
#define BATTERY_ALPHA               0.10f   /* EMA da tensao filtrada           */
#define BATTERY_TOOTH_DIVIDER       32      /* 1 amostra a cada N dentes do CKP */

/* Se 1, o modulo consulta MAP_IsBusy() e cede o ADC1 ao MAP.
 * Deixe 1 sempre que os dois modulos convivem no mesmo ADC. */
#define BATTERY_YIELD_TO_MAP        1

typedef struct {
	float voltage;
	uint16_t raw_adc;
} Calibration_t;

typedef struct {
	/* leitura instantanea */
	float voltage;                  /* V da ultima conversao valida */
	uint16_t raw_adc;

	/* leitura filtrada - use esta para dead time de injetor */
	float voltage_filtered;

	/* reta de conversao: V por LSB */
	float linear_cte;

	Calibration_t lvL;
	Calibration_t lvH;

	/* diagnostico */
	bool valid;
	uint16_t fault_count;
} Battery_t;

extern Battery_t battery;

void BATTERY_Init(void);
void BATTERY_LoadDefaultCurve(void);

uint16_t BATTERY_ReadRaw(void);
float BATTERY_ReadVoltage(void);        /* media de BATTERY_CAL_SAMPLES */
float BATTERY_VoltageFromRaw(uint16_t raw);

void BATTERY_OnCrankTooth(void);
void BATTERY_Poll(void);

float BATTERY_GetVoltage(void);         /* V filtrada, com fallback seguro */
float BATTERY_GetInstant(void);
bool BATTERY_IsValid(void);
bool BATTERY_IsBusy(void);

/* --- calibracao de dois pontos --- */
bool BATTERY_Calibrate_Low(float known_v);
bool BATTERY_Calibrate_High(float known_v);

#endif /* INC_BATTERY_MANAGER_H_ */
