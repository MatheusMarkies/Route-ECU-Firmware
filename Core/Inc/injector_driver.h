/*
 * injector_driver.h
 *
 * Created on: 20 de ago de 2025
 * Author: Matheus Markies
 *
 * Driver para o controlador de injetor LM1949.
 * Utiliza timers de hardware para gerar pulsos de injeção precisos.
 */

#ifndef INC_INJECTOR_DRIVER_H_
#define INC_INJECTOR_DRIVER_H_

#include "main.h"
#include <stdbool.h>

extern const long TIMER_CLOCK_FREQ_HZ;

extern TIM_HandleTypeDef htim1;

// --- Mapeamento dos Timers/Canais para cada injetor ---
#define INJECTOR_1_CHANNEL    TIM_CHANNEL_1
#define INJECTOR_2_CHANNEL    TIM_CHANNEL_2
#define INJECTOR_3_CHANNEL    TIM_CHANNEL_3
#define INJECTOR_4_CHANNEL    TIM_CHANNEL_4

/**
 * @brief Dispara um único pulso de teste num injetor específico.
 * @note  Esta função é para fins de diagnóstico e ignora a lógica principal do motor.
 * @param cylinder_index O índice do injetor a testar (0 a 3).
 * @param pulse_width_ms A largura de pulso desejada para o teste, em milissegundos.
 */
void Injector_TestPulse(uint8_t cylinder_index, float pulse_width_ms);

/**
 * @brief Inicializa o módulo de injeção.
 * @note Configura e inicia os timers de hardware para todos os canais de injetor.
 * Deve ser chamado uma vez na função de setup.
 */
void Injector_Init(void);

/**
 * @brief Agenda um pulso de injeção para um cilindro específico.
 * @param cylinder_index O índice do cilindro a injetar (ex: 0 para cilindro 1).
 * @param pulse_width_ms A duração total que o injetor deve permanecer aberto, em milissegundos.
 */
void Injector_SchedulePulse(uint8_t cylinder_index, float pulse_width_ms);

#endif /* INC_INJECTOR_DRIVER_H_ */
