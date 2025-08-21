/*
 * ignition_driver.h
 *
 *  Created on: Aug 20, 2025
 *      Author: Matheus Markies
 */

#ifndef INC_IGNITION_DRIVER_H_
#define INC_IGNITION_DRIVER_H_

#include "main.h"
#include "ecu_config.h"

extern const long TIMER_CLOCK_FREQ_HZ;

extern TIM_HandleTypeDef htim1;

// Mapeamento dos canais
#define IGNITION_COIL_1_CHANNEL TIM_CHANNEL_1
#define IGNITION_COIL_2_CHANNEL TIM_CHANNEL_2
#define IGNITION_COIL_3_CHANNEL TIM_CHANNEL_3
#define IGNITION_COIL_4_CHANNEL TIM_CHANNEL_4

void Ignition_Init(void);
void Ignition_ScheduleSpark(uint8_t, float, float);
uint32_t DegreesToTimeUs(float);
uint32_t ApplyDwellCompensation(float, float);

#endif /* INC_IGNITION_DRIVER_H_ */
