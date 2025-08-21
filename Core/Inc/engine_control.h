/*
 * engine_controll.h
 *
 *  Created on: Aug 19, 2025
 *      Author: Matheus Markies
 */

#ifndef INC_ENGINE_CONTROL_H_
#define INC_ENGINE_CONTROL_H_

#include "main.h"
#include "injector_driver.h"
#include "ignition_driver.h"

extern const long TIMER_CLOCK_FREQ_HZ;

void EngineControl_Init(void);
void Task_EngineControl(void const * argument);
void EngineControl_CrankEventCallback(void);

#endif /* INC_ENGINE_CONTROL_H_ */
