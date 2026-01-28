/*
 * ADC_driver.h
 *
 *  Created on: 27 de jan. de 2026
 *      Author: Matheus Markies
 */

#ifndef INC_ADC_DRIVER_H_
#define INC_ADC_DRIVER_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// Endereços I2C dos ADCs (7-bit)
#define AD7998_U16_ADDR    0x23  // AD7998 U16 (AS=AGND)
#define AD7998_U17_ADDR    0x24  // AD7998 U17 (AS=VDD)

// Registradores internos do AD7998
#define AD7998_REG_CONVERSION_RESULT   0x00
#define AD7998_REG_ALERT_STATUS        0x01
#define AD7998_REG_CONFIGURATION       0x02
#define AD7998_REG_CYCLE_TIMER         0x03

// Resolução do AD7998
#define AD7998_RESOLUTION_BITS  12
#define AD7998_MAX_VALUE        4095.0f

// ============================================================================
// ENUMERAÇÕES PARA MAPEAMENTO DE SENSORES
// ============================================================================

// Sensores conectados ao ADC U16 (0x23)
typedef enum {
    SENSOR_OUT_7 = 0,      // VIN1 - Canal 1
    SENSOR_OUT_1 = 1,      // VIN2 - Canal 2
    SENSOR_OUT_8 = 2,      // VIN3 - Canal 3
    SENSOR_OUT_2 = 3,      // VIN4 - Canal 4
    SENSOR_OUT_9 = 4,      // VIN5 - Canal 5
    SENSOR_OUT_3 = 5,      // VIN6 - Canal 6
    SENSOR_OPT_2_OUT = 6,  // VIN7 - Canal 7
    SENSOR_OPT_3_OUT = 7,  // VIN8 - Canal 8
    SENSOR_U16_COUNT = 8
} SensorU16_t;

// Sensores conectados ao ADC U17 (0x24)
typedef enum {
    SENSOR_OUT_10 = 0,      // VIN1 - Canal 1
    SENSOR_OUT_4 = 1,       // VIN2 - Canal 2
    SENSOR_OUT_11 = 2,      // VIN3 - Canal 3
    SENSOR_OUT_5 = 3,       // VIN4 - Canal 4
    SENSOR_OUT_12 = 4,      // VIN5 - Canal 5
    SENSOR_OUT_6 = 5,       // VIN6 - Canal 6
    SENSOR_OPT_0_OUT = 6,   // VIN7 - Canal 7
    SENSOR_OPT_1_OUT = 7,   // VIN8 - Canal 8
    SENSOR_U17_COUNT = 8
} SensorU17_t;

// ============================================================================
// ESTRUTURAS DE DADOS
// ============================================================================

// Estrutura para armazenar dados de um ADC
typedef struct {
    uint8_t i2c_address;              // Endereço I2C do ADC
    I2C_HandleTypeDef *hi2c;          // Ponteiro para interface I2C
    uint16_t raw_values[8];           // Valores brutos (0-4095) de cada canal
    float voltages[8];                // Tensões calculadas (V) de cada canal
    float vref;                       // Tensão de referência (V)
    bool is_initialized;              // Flag de inicialização
    uint32_t last_read_time;          // Timestamp da última leitura
} AD7998_Device_t;

// ============================================================================
// VARIÁVEIS GLOBAIS
// ============================================================================

extern AD7998_Device_t adc_u16;  // ADC U16 (endereço 0x23)
extern AD7998_Device_t adc_u17;  // ADC U17 (endereço 0x24)

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa os dois ADCs AD7998
 * @param hi2c2 Ponteiro para a interface I2C2
 * @param vref Tensão de referência em volts (ex: 3.3V)
 * @return HAL_OK se sucesso, HAL_ERROR se falha
 */
HAL_StatusTypeDef AD7998_Init(I2C_HandleTypeDef *hi2c2, float vref);

/**
 * @brief Lê um canal específico do ADC U16
 * @param sensor Enumeração do sensor (SENSOR_OUT_7, SENSOR_OUT_1, etc)
 * @param raw_value Ponteiro para armazenar valor bruto (0-4095)
 * @param voltage Ponteiro para armazenar tensão calculada (V)
 * @return HAL_OK se sucesso, HAL_ERROR se falha
 */
HAL_StatusTypeDef AD7998_U16_ReadSensor(SensorU16_t sensor, uint16_t *raw_value, float *voltage);

/**
 * @brief Lê um canal específico do ADC U17
 * @param sensor Enumeração do sensor (SENSOR_OUT_10, SENSOR_OUT_4, etc)
 * @param raw_value Ponteiro para armazenar valor bruto (0-4095)
 * @param voltage Ponteiro para armazenar tensão calculada (V)
 * @return HAL_OK se sucesso, HAL_ERROR se falha
 */
HAL_StatusTypeDef AD7998_U17_ReadSensor(SensorU17_t sensor, uint16_t *raw_value, float *voltage);

/**
 * @brief Lê todos os 8 canais do ADC U16
 * @return HAL_OK se sucesso, HAL_ERROR se falha
 */
HAL_StatusTypeDef AD7998_U16_ReadAllChannels(void);

/**
 * @brief Lê todos os 8 canais do ADC U17
 * @return HAL_OK se sucesso, HAL_ERROR se falha
 */
HAL_StatusTypeDef AD7998_U17_ReadAllChannels(void);

/**
 * @brief Converte valor bruto (0-4095) para tensão
 * @param raw_value Valor bruto do ADC
 * @param vref Tensão de referência
 * @return Tensão em volts
 */
static inline float AD7998_RawToVoltage(uint16_t raw_value, float vref) {
    return (raw_value * vref) / AD7998_MAX_VALUE;
}

/**
 * @brief Retorna string com nome do sensor U16
 */
const char* AD7998_U16_GetSensorName(SensorU16_t sensor);

/**
 * @brief Retorna string com nome do sensor U17
 */
const char* AD7998_U17_GetSensorName(SensorU17_t sensor);

#endif /* INC_ADC_DRIVER_H_ */
