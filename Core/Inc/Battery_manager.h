/*
 * Battery_manager.h
 *
 *  Created on: 28 de jan. de 2026
 *      Author: Matheus Markies
 */

#ifndef INC_BATTERY_MANAGER_H_
#define INC_BATTERY_MANAGER_H_

#include "main.h"

#define ADC_RESOLUTION 12
#define ADC_MAX 4095.0f
#define VREF 3.3f

// R2 (10k) -> ADC -> R1 (47k) -> GND
#define R1 47000.0f
#define R2 10000.0f
#define VOLTAGE_DIVIDER_FACTOR (R2/(R1 + R2))

#define BATTERY_SAMPLES 20

#define MAX_BATTERY_VOLTAGE 18.35f
#define MIN_BATTERY_VOLTAGE 0.0f

typedef struct {
    float voltage;
    uint16_t raw_adc;
} Calibration_t;

typedef struct {
    float voltage;
    uint16_t raw_adc;

    float linear_cte;

    Calibration_t lvL;
    Calibration_t lvH;
} Battery_t;

extern Battery_t battery;

void BATTERY_Init(void);
uint16_t BATTERY_ReadRaw(void);
float BATTERY_ReadVoltage(void);
void BATTERY_ReadVoltageFiltered(void);
void BATTERY_Calibrate(void);
void BATTERY_Calibrate_LV(float value);

#endif /* INC_BATTERY_MANAGER_H_ */
