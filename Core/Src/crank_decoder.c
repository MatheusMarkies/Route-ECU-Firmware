/*
 * crank_decoder.c
 *
 *  Created on: Aug 20, 2025
 *      Author: Matheus Markies
 */


/*
 * crank_decoder.c
 *
 * Módulo para decodificar o sinal do virabrequim (CKP) usando um timer em modo Input Capture.
 */

#include "crank_decoder.h"

void Crank_Init(void) {
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
}

// ESTA FUNÇÃO É CHAMADA PELA ISR DO TIMER
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) { // Confirma que é o timer correto
        uint32_t current_capture_tick = HAL_TIM_GetCapturedValue(htim, TIM_CHANNEL_1);

        if (last_capture_tick != 0) {
            // --- 1. Calcular delta de ticks considerando overflow ---
            uint32_t dt_ticks;
            if (current_capture_tick >= last_capture_tick) {
                dt_ticks = current_capture_tick - last_capture_tick;
            } else {
                // Overflow do timer
                dt_ticks = (0xFFFF - last_capture_tick) + current_capture_tick + 1;
            }

            // --- 2. Detectar Missing Tooth para sincronização ---
            if (!engine_is_synced && last_dt_ticks > 0) {
                if ((float)dt_ticks > 1.5f * (float)last_dt_ticks) {
                    engine_is_synced = 1;
                    tooth_counter = 0;
                    // Exemplo: sincronismo a 90 graus antes do PMS do cilindro 1
                    g_engineState.engine_angle_deg = 360.0f - 90.0f;
                }
            }

            // --- 3. Calcular RPM ---
            float time_per_tooth_s = (float)dt_ticks / (float)TIMER_CLOCK_FREQ_HZ;
            if (time_per_tooth_s > 0.0f) {
                g_engineState.rpm = (uint32_t)((60.0f / time_per_tooth_s) / g_ecuConfig.ckp.num_teeth_on_wheel);
            }

            last_dt_ticks = dt_ticks;

            // --- 4. Atualizar ângulo do motor ---
            if (engine_is_synced) {
                float degrees_per_tooth = 360.0f / g_ecuConfig.ckp.num_teeth_on_wheel;
                g_engineState.engine_angle_deg += degrees_per_tooth;
                tooth_counter++;

                // Manter ângulo dentro de 0-720 graus
                if (g_engineState.engine_angle_deg >= 720.0f) {
                    g_engineState.engine_angle_deg -= 720.0f;
                }
            }

            // --- 5. Chamar callback para tarefa de controle ---
            EngineControl_CrankEventCallback();
        }

        last_capture_tick = current_capture_tick;
    }
}
