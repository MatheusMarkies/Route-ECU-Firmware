/*
 * Battery_manager.c
 *
 *  Created on: 28 de jan. de 2026
 *  Revisado em: 10 de ago. de 2026
 *      Author: Matheus Markies
 */

#include "Battery_manager.h"
#include <stdio.h>

#if BATTERY_YIELD_TO_MAP
#include "MAP_sensor.h"
#endif

extern ADC_HandleTypeDef hadc1;

Battery_t battery = { .voltage = 0.0f, .raw_adc = 0, .voltage_filtered =
		BATTERY_DEFAULT_VOLTAGE, .linear_cte = 0.0f, .lvL = { .voltage = 0.0f,
		.raw_adc = 0 }, .lvH = { .voltage = 0.0f, .raw_adc = 0 }, .valid = false,
		.fault_count = 0 };

static volatile bool conversion_pending = false;
static volatile uint8_t pending_ticks = 0;
static uint16_t tooth_counter = 0;
static bool ema_seeded = false;

static bool BATTERY_SelectChannel(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	sConfig.Channel = BATTERY_ADC_CHANNEL;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = BATTERY_ADC_SAMPLETIME;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;

	return (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK);
}

static void BATTERY_UpdateLinearCte(void) {
	float delta_v = battery.lvH.voltage - battery.lvL.voltage;
	float delta_adc = (float) battery.lvH.raw_adc - (float) battery.lvL.raw_adc;

	if (delta_adc > 1.0f) {
		battery.linear_cte = delta_v / delta_adc;
	}
}

static void BATTERY_CountFault(void) {
	if (battery.fault_count < BATTERY_FAULT_LIMIT) {
		battery.fault_count++;
	} else {
		battery.valid = false;
	}
}

/* Converte, valida e publica. Retorna false em leitura implausivel. */
static bool BATTERY_Process(uint16_t raw) {
	if (raw == 0xFFFF) {
		BATTERY_CountFault();
		return false;
	}

	battery.raw_adc = raw;

	float v = BATTERY_VoltageFromRaw(raw);

	bool rail_fault = (raw <= BATTERY_RAW_OPEN) || (raw >= BATTERY_RAW_SHORT);
	bool range_fault = (v < BATTERY_MIN_VOLTAGE) || (v > BATTERY_MAX_VOLTAGE);

	if (rail_fault || range_fault) {
		BATTERY_CountFault();
		return false;
	}

	battery.fault_count = 0;
	battery.valid = true;
	battery.voltage = v;

	if (!ema_seeded) {
		battery.voltage_filtered = v;
		ema_seeded = true;
	} else {
		battery.voltage_filtered += BATTERY_ALPHA
				* (v - battery.voltage_filtered);
	}

	return true;
}

/* Dispara uma conversao. Nao espera. */
static void BATTERY_TryStart(void) {
	if (BATTERY_SelectChannel() && HAL_ADC_Start(&hadc1) == HAL_OK) {
		conversion_pending = true;
		pending_ticks = 0;
	}
}

/* Coleta o resultado se o EOC ja subiu. Aborta se ficar preso. */
static void BATTERY_TryCollect(void) {
	if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) {
		uint16_t raw = (uint16_t) HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		conversion_pending = false;
		pending_ticks = 0;
		BATTERY_Process(raw);
		return;
	}

	if (pending_ticks < BATTERY_PENDING_TIMEOUT) {
		pending_ticks++;
		return;
	}

	/* Conversao presa: o MAP reconfigurou o canal e engoliu o EOC.
	 * Aborta e recomeca do zero na proxima janela. */
	HAL_ADC_Stop(&hadc1);
	conversion_pending = false;
	pending_ticks = 0;
	BATTERY_CountFault();
}

/* Media bloqueante - so para calibracao e leitura com motor parado. */
static uint16_t BATTERY_ReadRawAveraged(uint16_t samples) {
	uint32_t sum = 0; /* NAO usar uint16_t: 32 x 4095 estoura 65535 */
	uint16_t taken = 0;

	for (uint16_t i = 0; i < samples; i++) {
		uint16_t raw = BATTERY_ReadRaw();
		if (raw != 0xFFFF) {
			sum += raw;
			taken++;
		}
		HAL_Delay(2);
	}

	if (taken == 0) {
		return 0xFFFF;
	}

	return (uint16_t) (sum / taken);
}

void BATTERY_LoadDefaultCurve(void) {
	battery.lvL.voltage = 0.0f;
	battery.lvL.raw_adc = 0;

	battery.lvH.voltage = BATTERY_FULL_SCALE_V; /* 18,81 V com 47k/10k */
	battery.lvH.raw_adc = (uint16_t) BATTERY_ADC_MAX;

	BATTERY_UpdateLinearCte();
}

void BATTERY_Init(void) {
	/* Unico ponto do projeto que calibra o ADC1. O MAP depende disto. */
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
			ADC_SINGLE_ENDED) != HAL_OK) {
		Error_Handler();
	}

	BATTERY_LoadDefaultCurve();

	battery.voltage = 0.0f;
	battery.voltage_filtered = BATTERY_DEFAULT_VOLTAGE;
	battery.raw_adc = 0;
	battery.valid = false;
	battery.fault_count = 0;

	conversion_pending = false;
	pending_ticks = 0;
	tooth_counter = 0;
	ema_seeded = false;
}

uint16_t BATTERY_ReadRaw(void) {
	uint16_t adc_value = 0xFFFF;

	if (conversion_pending) {
		HAL_ADC_Stop(&hadc1);
		conversion_pending = false;
		pending_ticks = 0;
	}

	if (!BATTERY_SelectChannel()) {
		return 0xFFFF;
	}

	if (HAL_ADC_Start(&hadc1) != HAL_OK) {
		return 0xFFFF;
	}

	if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) {
		adc_value = (uint16_t) HAL_ADC_GetValue(&hadc1);
	}

	HAL_ADC_Stop(&hadc1);

	return adc_value;
}

float BATTERY_VoltageFromRaw(uint16_t raw) {
	return battery.lvL.voltage
			+ (battery.linear_cte
					* ((float) raw - (float) battery.lvL.raw_adc));
}

float BATTERY_ReadVoltage(void) {
	BATTERY_Process(BATTERY_ReadRawAveraged(BATTERY_CAL_SAMPLES));
	return battery.voltage_filtered;
}

void BATTERY_OnCrankTooth(void) {
#if BATTERY_YIELD_TO_MAP
	if (MAP_IsBusy()) {
		return; /* o MAP tem prioridade no ADC1 */
	}
#endif

	if (conversion_pending) {
		BATTERY_TryCollect();
		return;
	}

	if (++tooth_counter >= BATTERY_TOOTH_DIVIDER) {
		tooth_counter = 0;
		BATTERY_TryStart();
	}
}

void BATTERY_Poll(void) {
#if BATTERY_YIELD_TO_MAP
	if (MAP_IsBusy()) {
		return;
	}
#endif

	if (conversion_pending) {
		BATTERY_TryCollect();
	} else {
		BATTERY_TryStart();
	}
}

float BATTERY_GetVoltage(void) {
	if (!battery.valid) {
		return BATTERY_DEFAULT_VOLTAGE;
	}
	return battery.voltage_filtered;
}

float BATTERY_GetInstant(void) {
	return battery.voltage;
}

bool BATTERY_IsValid(void) {
	return battery.valid;
}

bool BATTERY_IsBusy(void) {
	return conversion_pending;
}

/* ==========================================================================
 *  CALIBRACAO DE DOIS PONTOS
 *
 *  Aplique uma tensao conhecida na entrada Bat_Voltage (fonte de bancada,
 *  medida com multimetro) e chame a funcao com esse valor.
 *
 *      BATTERY_Calibrate_Low(10.0f);    // ponto baixo
 *      BATTERY_Calibrate_High(14.5f);   // ponto alto, recalcula o ganho
 *
 *  Use dois pontos dentro da faixa de trabalho real (10 a 15 V), nao
 *  0 V e fundo de escala: extrapolar de longe amplifica o erro de offset.
 * ========================================================================== */

bool BATTERY_Calibrate_Low(float known_v) {
	uint16_t raw = BATTERY_ReadRawAveraged(BATTERY_CAL_SAMPLES);

	if (raw == 0xFFFF || raw >= BATTERY_RAW_SHORT) {
		printf("[BATTERY] ERRO: leitura invalida na calibracao (raw=%u)\r\n",
				raw);
		return false;
	}

	battery.lvL.voltage = known_v;
	battery.lvL.raw_adc = raw;

	BATTERY_UpdateLinearCte();

	printf("[BATTERY] Ponto baixo: %.3f V @ raw %u\r\n", known_v, raw);
	return true;
}

bool BATTERY_Calibrate_High(float known_v) {
	uint16_t raw = BATTERY_ReadRawAveraged(BATTERY_CAL_SAMPLES);

	if (raw == 0xFFFF) {
		printf("[BATTERY] ERRO: leitura invalida na calibracao\r\n");
		return false;
	}

	battery.lvH.voltage = known_v;
	battery.lvH.raw_adc = raw;

	float delta_voltage = battery.lvH.voltage - battery.lvL.voltage;
	float delta_adc = (float) battery.lvH.raw_adc - (float) battery.lvL.raw_adc;

	printf("[BATTERY] Delta voltage: %.4f V\r\n", delta_voltage);
	printf("[BATTERY] Delta ADC: %.4f\r\n", delta_adc);

	if (delta_adc <= 1.0f) {
		printf("[BATTERY] ERRO: Delta ADC <= 1, pontos muito proximos\r\n");
		return false;
	}

	BATTERY_UpdateLinearCte();
	printf("[BATTERY] linear_cte calculado: %.6f V/LSB\r\n",
			battery.linear_cte);

	return true;
}
