/*
 * ignition_driver.c
 *
 * Created on: 20 de ago de 2025
 * Author: Matheus Markies
 */

#include "ignition_driver.h"
#include "main.h"

// --- Definição das Variáveis Globais ---
extern EcuConfig_t  g_ecuConfig;
extern EngineData_t g_engineData;
extern EngineState_t g_engineState;

void Ignition_Init(void) {
    HAL_TIM_PWM_Start(&htim4, IGNITION_COIL_1_CHANNEL);
    HAL_TIM_PWM_Start(&htim4, IGNITION_COIL_2_CHANNEL);
    HAL_TIM_PWM_Start(&htim4, IGNITION_COIL_3_CHANNEL);
    HAL_TIM_PWM_Start(&htim4, IGNITION_COIL_4_CHANNEL);

    __HAL_TIM_SET_COMPARE(&htim4, IGNITION_COIL_1_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim4, IGNITION_COIL_2_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim4, IGNITION_COIL_3_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim4, IGNITION_COIL_4_CHANNEL, 0);
}

// ============================================================
// Agenda o disparo da faísca
// ============================================================
void Ignition_ScheduleSpark(uint8_t cylinder_index, float advance_deg, float dwell_ms) {
    if (g_engineState.rpm < 200) return;

    uint32_t time_per_degree_us = DegreesToTimeUs(1.0f); // µs por grau
    uint32_t dwell_us = ApplyDwellCompensation(dwell_ms, g_engineData.battery_voltage) * 1000.0f;
    uint32_t advance_us = (uint32_t)(advance_deg * time_per_degree_us);

    uint32_t dwell_start_delay = advance_us + dwell_us;

    uint32_t pulse_ticks = dwell_us;
    uint32_t delay_ticks = dwell_start_delay;

    switch (cylinder_index) {
        case 0:
            __HAL_TIM_SET_AUTORELOAD(&htim4, delay_ticks + pulse_ticks);
            __HAL_TIM_SET_COMPARE(&htim4, IGNITION_COIL_1_CHANNEL, delay_ticks);
            break;
        case 1:
            __HAL_TIM_SET_AUTORELOAD(&htim4, delay_ticks + pulse_ticks);
            __HAL_TIM_SET_COMPARE(&htim4, IGNITION_COIL_2_CHANNEL, delay_ticks);
            break;
        case 2:
            __HAL_TIM_SET_AUTORELOAD(&htim4, delay_ticks + pulse_ticks);
            __HAL_TIM_SET_COMPARE(&htim4, IGNITION_COIL_3_CHANNEL, delay_ticks);
            break;
        case 3:
            __HAL_TIM_SET_AUTORELOAD(&htim4, delay_ticks + pulse_ticks);
            __HAL_TIM_SET_COMPARE(&htim4, IGNITION_COIL_4_CHANNEL, delay_ticks);
            break;
    }
}

/* ... (conteúdo anterior, incluindo a função Ignition_ScheduleSpark) ... */

// --- Implementação da Função de Debug ---

/**
 * @brief Dispara um único pulso de teste numa bobina de ignição específica.
 */
void Ignition_TestPulse(uint8_t cylinder_index, float dwell_ms) {
    // Validação de segurança básica
    if (cylinder_index >= 4 || dwell_ms <= 0.0f || dwell_ms > 10.0f) {
        return; // Parâmetros inválidos
    }

    // 1. Converte o dwell em milissegundos para ticks do nosso timer (que são microssegundos)
    // Usamos a constante TIMER_CLOCK_FREQ_HZ definida no cabeçalho.
    uint32_t pulse_ticks = (uint32_t)(dwell_ms * 1000.0f);

    // 2. Seleciona o canal do timer correto e dispara um único pulso
    switch (cylinder_index) {
        case 0:
            // Configura o Período (ARR) e o Compare (CCR) para gerar um único pulso
            // com a duração exata do dwell.
            htim4.Instance->ARR = pulse_ticks + 10; // Período ligeiramente maior para garantir o pulso completo
            htim4.Instance->CCR1 = pulse_ticks;     // Largura do pulso para o Canal 1
            // Inicia o timer em modo "One-Pulse". O hardware gera o pulso e para automaticamente.
            HAL_TIM_OnePulse_Start(&htim4, IGNITION_COIL_1_CHANNEL);
            break;
        case 1:
            htim4.Instance->ARR = pulse_ticks + 10;
            htim4.Instance->CCR2 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim4, IGNITION_COIL_2_CHANNEL);
            break;
        case 2:
            htim4.Instance->ARR = pulse_ticks + 10;
            htim4.Instance->CCR3 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim4, IGNITION_COIL_3_CHANNEL);
            break;
        case 3:
            htim4.Instance->ARR = pulse_ticks + 10;
            htim4.Instance->CCR4 = pulse_ticks;
            HAL_TIM_OnePulse_Start(&htim4, IGNITION_COIL_4_CHANNEL);
            break;
    }
}

// ============================================================
// Callbacks de segurança (desliga PWM depois do pulso)
// ============================================================
void Ignition_TimerCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        __HAL_TIM_SET_COMPARE(&htim4, IGNITION_COIL_1_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim4, IGNITION_COIL_2_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim4, IGNITION_COIL_3_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim4, IGNITION_COIL_4_CHANNEL, 0);
    }
}

// ============================================================
// Funções auxiliares
// ============================================================

// Converte graus de rotação em microssegundos
uint32_t DegreesToTimeUs(float degrees) {
    float rev_per_ms = (float)g_engineState.rpm / 60000.0f;
    float time_per_rev_ms = 1.0f / rev_per_ms;
    float time_per_deg_ms = time_per_rev_ms / 360.0f;
    return (uint32_t)(degrees * time_per_deg_ms * 1000.0f);
}

// Compensa o dwell pela tensão da bateria (tabela simplificada)
uint32_t ApplyDwellCompensation(float dwell_ms, float vbat) {
    if (vbat < 10.0f) return dwell_ms * 1.3f;
    if (vbat < 12.0f) return dwell_ms * 1.1f;
    if (vbat > 14.5f) return dwell_ms * 0.9f;
    return dwell_ms;
}
