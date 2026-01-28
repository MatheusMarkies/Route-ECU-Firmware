/*
 * Battery_manager.h
 *
 *  Created on: 28 de jan. de 2026
 *      Author: Matheus Markies
 */

#ifndef INC_BATTERY_MANAGER_H_
#define INC_BATTERY_MANAGER_H_

#include "stm32h7xx_hal.h"

#define R1_VALUE 47000.0f   // Resistor superior (47k ohm)
#define R2_VALUE 10000.0f    // Resistor inferior (10k ohm)
#define VREF 3.3f           // Tensão de referência do ADC (3.3V)
#define ADC_MAX 65535.0f     // Resolução 12-bit

#define VOLTAGE_DIVIDER_FACTOR ((R1_VALUE + R2_VALUE) / R2_VALUE)
#define BATTERY_SAMPLES 16

typedef struct {
    float voltage;
} Battery_t;

extern Battery_t battery;

uint16_t BATTERY_ReadRaw(void);
float BATTERY_ReadVoltage(void);
void BATTERY_ReadVoltageFiltered(void);

#endif /* INC_BATTERY_MANAGER_H_ */
