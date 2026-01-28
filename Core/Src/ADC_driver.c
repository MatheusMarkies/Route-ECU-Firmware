/*
 * ADC_driver.c
 *
 *  Created on: 27 de jan. de 2026
 *      Author: Matheus Markies
 */

#include "ADC_driver.h"
#include <stdio.h>
#include <string.h>

AD7998_Device_t adc_u16 = {0};
AD7998_Device_t adc_u17 = {0};

#define I2C_TIMEOUT_MS  100

static HAL_StatusTypeDef AD7998_WritePointer(AD7998_Device_t *dev, uint8_t reg_addr) {
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_address << 1,
                                   &reg_addr, 1, I2C_TIMEOUT_MS);
}

/**
 * @brief Lê um canal específico do ADC (Mode 2 - Command Mode)
 */
static HAL_StatusTypeDef AD7998_ReadChannel(AD7998_Device_t *dev, uint8_t channel,
                                            uint16_t *raw_value, float *voltage) {
    HAL_StatusTypeDef status;
    uint8_t cmd_byte;
    uint8_t rx_data[2];

    if (channel > 7) return HAL_ERROR;

    // Modo 2: Command bits para selecionar canal único
    // Bits C4-C1 = 1000 (CH1), 1001 (CH2), ..., 1111 (CH8)
    cmd_byte = 0x80 | ((channel + 1) << 4);  // 1000 + canal

    status = HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_address << 1,
                                     &cmd_byte, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) return status;

    HAL_Delay(1);

    status = HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_address << 1,
                                    rx_data, 2, I2C_TIMEOUT_MS);
    if (status != HAL_OK) return status;
    uint16_t raw = ((rx_data[0] & 0x0F) << 8) | rx_data[1];

    if (raw_value) *raw_value = raw;
    if (voltage) *voltage = AD7998_RawToVoltage(raw, dev->vref);

    dev->raw_values[channel] = raw;
    dev->voltages[channel] = AD7998_RawToVoltage(raw, dev->vref);
    dev->last_read_time = HAL_GetTick();

    return HAL_OK;
}

/**
 * @brief Configura o ADC para conversão sequencial de todos os canais
 */
static HAL_StatusTypeDef AD7998_ConfigureAllChannels(AD7998_Device_t *dev) {
    HAL_StatusTypeDef status;
    uint8_t config_data[3];

    // Configuration Register (0x02)
    config_data[0] = AD7998_REG_CONFIGURATION;

    // MSB: CH8-CH1 todos habilitados = 0xFF
    config_data[1] = 0xFF;

    // LSB: [FLTR=1][ALERT_EN=0][BUSY/ALERT=0][POLARITY=0] = 0x08
    config_data[2] = 0x08;

    status = HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_address << 1,
                                     config_data, 3, I2C_TIMEOUT_MS);
    return status;
}

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

HAL_StatusTypeDef AD7998_Init(I2C_HandleTypeDef *hi2c2, float vref) {
    HAL_StatusTypeDef status;

    // Configurar ADC (0x23)
    adc_u16.i2c_address = AD7998_U16_ADDR;
    adc_u16.hi2c = hi2c2;
    adc_u16.vref = vref;
    memset(adc_u16.raw_values, 0, sizeof(adc_u16.raw_values));
    memset(adc_u16.voltages, 0, sizeof(adc_u16.voltages));

    // Testar comunicação U16
    status = HAL_I2C_IsDeviceReady(hi2c2, AD7998_U16_ADDR << 1, 3, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf("ERROR: AD7998 U16 (0x23) not responding!\r\n");
        return HAL_ERROR;
    }

    // Configurar U16
    status = AD7998_ConfigureAllChannels(&adc_u16);
    if (status != HAL_OK) {
        printf("ERROR: Failed to configure AD7998 U16\r\n");
        return HAL_ERROR;
    }

    adc_u16.is_initialized = true;
    printf("AD7998 U16 (0x23) initialized successfully\r\n");

    // Configurar ADC U17 (0x24)
    adc_u17.i2c_address = AD7998_U17_ADDR;
    adc_u17.hi2c = hi2c2;
    adc_u17.vref = vref;
    memset(adc_u17.raw_values, 0, sizeof(adc_u17.raw_values));
    memset(adc_u17.voltages, 0, sizeof(adc_u17.voltages));

    // Testar comunicação U17
    status = HAL_I2C_IsDeviceReady(hi2c2, AD7998_U17_ADDR << 1, 3, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf("ERROR: AD7998 U17 (0x24) not responding!\r\n");
        return HAL_ERROR;
    }

    // Configurar U17
    status = AD7998_ConfigureAllChannels(&adc_u17);
    if (status != HAL_OK) {
        printf("ERROR: Failed to configure AD7998 U17\r\n");
        return HAL_ERROR;
    }

    adc_u17.is_initialized = true;
    printf("AD7998 U17 (0x24) initialized successfully\r\n");

    return HAL_OK;
}

HAL_StatusTypeDef AD7998_U16_ReadSensor(SensorU16_t sensor, uint16_t *raw_value, float *voltage) {
    if (!adc_u16.is_initialized) return HAL_ERROR;
    if (sensor >= SENSOR_U16_COUNT) return HAL_ERROR;

    return AD7998_ReadChannel(&adc_u16, sensor, raw_value, voltage);
}

HAL_StatusTypeDef AD7998_U17_ReadSensor(SensorU17_t sensor, uint16_t *raw_value, float *voltage) {
    if (!adc_u17.is_initialized) return HAL_ERROR;
    if (sensor >= SENSOR_U17_COUNT) return HAL_ERROR;

    return AD7998_ReadChannel(&adc_u17, sensor, raw_value, voltage);
}

HAL_StatusTypeDef AD7998_U16_ReadAllChannels(void) {
    HAL_StatusTypeDef status;

    if (!adc_u16.is_initialized) return HAL_ERROR;

    for (uint8_t ch = 0; ch < 8; ch++) {
        status = AD7998_ReadChannel(&adc_u16, ch, NULL, NULL);
        if (status != HAL_OK) return status;
        HAL_Delay(1); // Pequeno delay entre leituras
    }

    return HAL_OK;
}

HAL_StatusTypeDef AD7998_U17_ReadAllChannels(void) {
    HAL_StatusTypeDef status;

    if (!adc_u17.is_initialized) return HAL_ERROR;

    for (uint8_t ch = 0; ch < 8; ch++) {
        status = AD7998_ReadChannel(&adc_u17, ch, NULL, NULL);
        if (status != HAL_OK) return status;
        HAL_Delay(1);
    }

    return HAL_OK;
}

const char* AD7998_U16_GetSensorName(SensorU16_t sensor) {
    static const char* names[] = {
        "SENSOR_OUT_7", "SENSOR_OUT_1", "SENSOR_OUT_8", "SENSOR_OUT_2",
        "SENSOR_OUT_9", "SENSOR_OUT_3", "SENSOR_OPT_2_OUT", "SENSOR_OPT_3_OUT"
    };
    return (sensor < SENSOR_U16_COUNT) ? names[sensor] : "INVALID";
}

const char* AD7998_U17_GetSensorName(SensorU17_t sensor) {
    static const char* names[] = {
        "SENSOR_OUT_10", "SENSOR_OUT_4", "SENSOR_OUT_11", "SENSOR_OUT_5",
        "SENSOR_OUT_12", "SENSOR_OUT_6", "SENSOR_OPT_0_OUT", "SENSOR_OPT_1_OUT"
    };
    return (sensor < SENSOR_U17_COUNT) ? names[sensor] : "INVALID";
}
