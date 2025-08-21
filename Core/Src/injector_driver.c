/*
 * injector_driver.c
 *
 * Created on: 20 de ago de 2025
 * Author: Matheus Markies
 */

#include "injector_driver.h"
#include "main.h"

// ============================================================
// Inicialização
// ============================================================
void Injector_Init(void) {
    // Inicia PWM em todos os canais (com duty = 0)
    HAL_TIM_PWM_Start(&htim3, INJECTOR_1_CHANNEL);
    HAL_TIM_PWM_Start(&htim4, INJECTOR_2_CHANNEL);
    HAL_TIM_PWM_Start(&htim4, INJECTOR_3_CHANNEL);
    HAL_TIM_PWM_Start(&htim3, INJECTOR_4_CHANNEL);

    __HAL_TIM_SET_COMPARE(&htim3, INJECTOR_1_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim4, INJECTOR_2_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim4, INJECTOR_3_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim3, INJECTOR_4_CHANNEL, 0);
}

// ============================================================
// Agenda pulso de injeção (em ms)
// ============================================================
void Injector_SchedulePulse(uint8_t cylinder_index, float pulse_width_ms) {
    if (pulse_width_ms <= 0.05f) return; // ignora pulsos muito curtos

    // 1. Converte largura de pulso para ticks do timer
    uint32_t pulse_ticks = (uint32_t)(pulse_width_ms * 1000.0f *
                                      (TIMER_CLOCK_FREQ_HZ / 1000000UL));

    // 2. Configura timer/canal conforme cilindro
    switch (cylinder_index) {
        case 0: // Injetor 1
        	htim3.Instance->ARR = pulse_ticks + 20;
        	htim3.Instance->CCR1 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim3, INJECTOR_1_CHANNEL);
            break;

        case 1: // Injetor 2
        	htim4.Instance->ARR = pulse_ticks + 20;
        	htim4.Instance->CCR2 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim4, INJECTOR_2_CHANNEL);
            break;

        case 2: // Injetor 3
        	htim4.Instance->ARR = pulse_ticks + 20;
        	htim4.Instance->CCR1 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim4, INJECTOR_3_CHANNEL);
            break;

        case 3: // Injetor 4
        	htim3.Instance->ARR = pulse_ticks + 20;
        	htim3.Instance->CCR2 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim3, INJECTOR_4_CHANNEL);
            break;
    }
}

// ============================================================
// Callback opcional (segurança) -> zera duty após o pulso
// ============================================================
void Injector_TimerCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
    if (htim->Instance == TIM3) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    }
}
