/*
 * MAP_sensor.c
 *
 *  Created on: 10 de ago. de 2026
 *      Author: Matheus Markies
 */

#include "MAP_sensor.h"
#include <stdio.h>

extern ADC_HandleTypeDef hadc1;

MAP_t map = { .pressure = MAP_DEFAULT_BARO, .raw_adc = 0, .pressure_cycle =
		MAP_DEFAULT_BARO, .accumulator = 0.0f, .samples = 0, .baro =
		MAP_DEFAULT_BARO, .linear_cte = 0.0f, .calL = { .pressure = 0.0f,
		.raw_adc = 0 }, .calH = { .pressure = 0.0f, .raw_adc = 0 }, .valid =
		false, .fault_count = 0 };

static volatile bool conversion_pending = false;
static float supply_voltage = MAP_SUPPLY_NOMINAL;

/* ==========================================================================
 *  PRIVADOS
 * ========================================================================== */

static bool MAP_SelectChannel(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	sConfig.Channel = MAP_ADC_CHANNEL;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = MAP_ADC_SAMPLETIME;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;

	return (HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK);
}

/* Recalcula a reta a partir dos dois pontos de calibracao. */
static void MAP_UpdateLinearCte(void) {
	float delta_p = map.calH.pressure - map.calL.pressure;
	float delta_adc = (float) map.calH.raw_adc - (float) map.calL.raw_adc;

	if (delta_adc > 1.0f) {
		map.linear_cte = delta_p / delta_adc;
	}
}

/* raw teorico para uma pressao, pela transferencia do datasheet. */
static uint16_t MAP_RawFromPressure(float kPa) {
	float v_out = supply_voltage * (MAP_SENSOR_A * kPa + MAP_SENSOR_B);
	float v_pin = v_out * MAP_DIVIDER_FACTOR;
	float raw = (v_pin / MAP_VREF) * MAP_ADC_MAX;

	if (raw < 0.0f) {
		raw = 0.0f;
	} else if (raw > MAP_ADC_MAX) {
		raw = MAP_ADC_MAX;
	}

	return (uint16_t) raw;
}

/* Media bloqueante - so para calibracao, com o motor parado. */
static uint16_t MAP_ReadRawAveraged(uint16_t samples) {
	uint32_t sum = 0;
	uint16_t taken = 0;

	for (uint16_t i = 0; i < samples; i++) {
		uint16_t raw = MAP_ReadRaw();
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

/* Converte, valida e publica. Retorna false em leitura implausivel. */
static bool MAP_Process(uint16_t raw) {
	if (raw == 0xFFFF) {
		if (map.fault_count < MAP_FAULT_LIMIT) {
			map.fault_count++;
		} else {
			map.valid = false;
		}
		return false;
	}

	map.raw_adc = raw;

	float p = MAP_PressureFromRaw(raw);

	bool rail_fault = (raw <= MAP_RAW_OPEN) || (raw >= MAP_RAW_SHORT);
	bool range_fault = (p < MAP_MIN_KPA) || (p > MAP_MAX_KPA);

	if (rail_fault || range_fault) {
		if (map.fault_count < MAP_FAULT_LIMIT) {
			map.fault_count++;
		} else {
			map.valid = false;
		}
		return false;
	}

	map.fault_count = 0;
	map.valid = true;
	map.pressure = p;

	return true;
}

/* ==========================================================================
 *  CICLO DE VIDA
 * ========================================================================== */

void MAP_LoadDefaultCurve(void) {
	map.calL.pressure = MAP_MIN_KPA;
	map.calL.raw_adc = MAP_RawFromPressure(MAP_MIN_KPA);

	map.calH.pressure = MAP_MAX_KPA;
	map.calH.raw_adc = MAP_RawFromPressure(MAP_MAX_KPA);

	MAP_UpdateLinearCte();
}

void MAP_Init(void) {
	map.baro = MAP_DEFAULT_BARO;
	map.pressure = MAP_DEFAULT_BARO;
	map.pressure_cycle = MAP_DEFAULT_BARO;
	map.accumulator = 0.0f;
	map.samples = 0;
	map.fault_count = 0;
	map.valid = false;
	conversion_pending = false;
}

void MAP_SetSupplyVoltage(float volts) {
	if (volts > 4.0f && volts < 6.0f) {
		supply_voltage = volts;
	}
}

/* ==========================================================================
 *  LEITURA BLOQUEANTE
 * ========================================================================== */

uint16_t MAP_ReadRaw(void) {
	uint16_t adc_value = 0xFFFF;

	if (conversion_pending) {
		HAL_ADC_Stop(&hadc1);
		conversion_pending = false;
	}

	if (!MAP_SelectChannel()) {
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

float MAP_PressureFromRaw(uint16_t raw) {
	return map.calL.pressure
			+ (map.linear_cte * ((float) raw - (float) map.calL.raw_adc));
}

bool MAP_Update(void) {
	return MAP_Process(MAP_ReadRaw());
}

/* ==========================================================================
 *  CAMINHO RAPIDO  -  chamado de dentro do decodificador de virabrequim
 *
 *  Nao bloqueia: coleta o resultado disparado no dente anterior e ja arma
 *  a proxima conversao. Latencia de 1 dente, custo de ~2 us por chamada.
 * ========================================================================== */

void MAP_OnCrankTooth(void) {
	if (conversion_pending) {
		if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) {
			uint16_t raw = (uint16_t) HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			conversion_pending = false;

			if (MAP_Process(raw)) {
				map.accumulator += map.pressure;
				map.samples++;
			}
		} else {
			/* ainda convertendo: tenta de novo no proximo dente */
			return;
		}
	}

	if (MAP_SelectChannel() && HAL_ADC_Start(&hadc1) == HAL_OK) {
		conversion_pending = true;
	}
}

void MAP_OnCycleComplete(void) {
	if (map.samples > 0) {
		float avg = map.accumulator / (float) map.samples;
		map.pressure_cycle += MAP_CYCLE_ALPHA * (avg - map.pressure_cycle);
	}

	map.accumulator = 0.0f;
	map.samples = 0;
}

/* ==========================================================================
 *  CONSUMO
 * ========================================================================== */

float MAP_GetPressure(void) {
	if (!map.valid) {
		return map.baro; /* fallback: carga plena atmosferica */
	}
	return map.pressure_cycle;
}

float MAP_GetInstant(void) {
	return map.pressure;
}

bool MAP_IsValid(void) {
	return map.valid;
}

bool MAP_IsBusy(void) {
    return conversion_pending;
}
/* ==========================================================================
 *  CALIBRACAO
 *
 *  Ponto 1 (MAP_LearnBaro): motor parado, coletor na atmosferica.
 *      Desloca a reta sem mexer no ganho. E o unico ponto que da para
 *      fazer no carro, e da correcao de altitude de graca.
 *      Sao Carlos (~850 m) da ~91,5 kPa, nao 101,3.
 *
 *  Ponto 2 (MAP_Calibrate_High): bomba de vacuo ou compressor com
 *      manometro de referencia. So faz sentido na bancada.
 * ========================================================================== */

bool MAP_LearnBaro(float known_kPa) {
	uint16_t raw = MAP_ReadRawAveraged(MAP_CAL_SAMPLES);

	if (raw == 0xFFFF || raw <= MAP_RAW_OPEN || raw >= MAP_RAW_SHORT) {
		printf("[MAP] ERRO: leitura invalida na calibracao (raw=%u)\r\n", raw);
		return false;
	}

	/* mantem o ganho, so desloca o ponto de ancoragem */
	map.calL.pressure = known_kPa;
	map.calL.raw_adc = raw;

	map.baro = known_kPa;
	map.pressure_cycle = known_kPa;

	printf("[MAP] Baro aprendida: %.2f kPa @ raw %u\r\n", known_kPa, raw);
	return true;
}

bool MAP_Calibrate_High(float known_kPa) {
	uint16_t raw = MAP_ReadRawAveraged(MAP_CAL_SAMPLES);

	if (raw == 0xFFFF) {
		printf("[MAP] ERRO: leitura invalida na calibracao\r\n");
		return false;
	}

	map.calH.pressure = known_kPa;
	map.calH.raw_adc = raw;

	float delta_p = map.calH.pressure - map.calL.pressure;
	float delta_adc = (float) map.calH.raw_adc - (float) map.calL.raw_adc;

	printf("[MAP] Delta pressao: %.4f kPa\r\n", delta_p);
	printf("[MAP] Delta ADC: %.4f\r\n", delta_adc);

	if (delta_adc <= 1.0f) {
		printf("[MAP] ERRO: Delta ADC <= 1, pontos muito proximos\r\n");
		return false;
	}

	MAP_UpdateLinearCte();
	printf("[MAP] linear_cte calculado: %.6f kPa/LSB\r\n", map.linear_cte);

	return true;
}
