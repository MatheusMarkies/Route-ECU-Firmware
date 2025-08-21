/*
 * engine_sensors.h
 *
 *  Created on: Aug 19, 2025
 *      Author: Matheus Markies
 */

#ifndef INC_ENGINE_SENSORS_H_
#define INC_ENGINE_SENSORS_H_

#include "main.h"
#include "ecu_config.h"

// Declaração da handle do ADC, para que o main.c possa passá-la para nós.
extern ADC_HandleTypeDef hadc1; // Supondo que ADC1 é para os sensores
extern ADC_HandleTypeDef hadc2; // Supondo que ADC2 é para a bateria

// --- Definição das Variáveis Globais ---
extern EcuConfig_t  g_ecuConfig;
extern EngineData_t g_engineData;
extern EngineState_t g_engineState;

#define ADC_MAX_VALUE 4095.0f
#define ADC_VREF      3.3f
#define NUM_SENSOR_CHANNELS 12

const float BATT_DIVIDER_RATIO = 18.81f / 3.3f;
const float SENSOR_DIVIDER_RATIO = 5.0f / 3.205f;

uint16_t g_adc_dma_buffer_sensors[NUM_SENSOR_CHANNELS];
uint16_t g_adc_dma_buffer_battery[1];
volatile int g_adc_conversion_complete = 0;

// Arrays globais para armazenamento genérico
uint16_t g_raw_adc_values[12];
float    g_sensor_voltages[12];
float    g_battery_adc_voltage;

// Arrays para armazenamento genérico, como no seu rascunho
float g_sensor_voltages[12];
float g_battery_adc_voltage;

void EngineSensors_Init(void);
void Task_ReadSensors(void const * argument);

#endif /* INC_ENGINE_SENSORS_H_ */
