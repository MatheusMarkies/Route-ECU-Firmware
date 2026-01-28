/*
 * MAX9924_driver.h
 *
 *  Created on: 27 de jan. de 2026
 *      Author: Matheus Markies
 */

#ifndef INC_MAX9924_DRIVER_H_
#define INC_MAX9924_DRIVER_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

//Resolucao dos Input Captures usando o pre escaler de 124 e HCLK de 125MHz: CK_CNT = HCLK / (PSC+1) = 1000000 Hz ~ 0.001ms

// Tipo de sensor VR
typedef enum {
    SENSOR_CKP = 0,
    SENSOR_CMP = 1,
    SENSOR_COUNT = 2
} VR_Sensor_Type_t;

// Estado da leitura
typedef enum {
    VR_STATE_LOW = 0,   // Sinal em nível baixo
    VR_STATE_HIGH = 1   // Sinal em nível alto dente detectado
} VR_State_t;

// Evento detectado
typedef enum {
    VR_EVENT_NONE = 0,      // Nenhum evento
    VR_EVENT_RISING = 1,    // Borda de subida (início do dente)
    VR_EVENT_FALLING = 2    // Borda de descida (fim do dente)
} VR_Event_t;

typedef struct {
    // Configuração
    GPIO_TypeDef *gpio_port;        // Porta GPIO do pino COUT
    uint16_t gpio_pin;              // Pino GPIO do COUT
    VR_Sensor_Type_t type;          // Tipo do sensor

    // Contadores
    uint32_t pulse_count;           // Contador total de pulsos
    uint32_t pulse_count_per_rev;   // Pulsos por revolução (configurável)
    uint32_t revolution_count;      // Contador de revoluções

    // Timing
    uint32_t last_edge_time;        // Timestamp da última borda (ms)
    uint32_t current_edge_time;        // Timestamp da última borda (ms)
    uint32_t pulse_width;           // Largura do último pulso (ms)
    uint32_t period;                // Período entre pulsos (ms)

    // RPM e velocidade
    float rpm;                      // Rotações por minuto calculadas
    float frequency_hz;             // Frequência em Hz

    uint16_t tooths;

    uint32_t timeout_ms;            // Timeout para considerar sinal inválido
    bool isSync;

} VR_Sensor_t;

extern VR_Sensor_t ckp_sensor;  // Sensor de virabrequim (CKP)
extern VR_Sensor_t cmp_sensor;  // Sensor de comando (CMP)
extern TIM_HandleTypeDef htim5;

#define CKP_CHANNEL TIM_CHANNEL_1
#define CMP_CHANNEL TIM_CHANNEL_2

#define CKP_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_1
#define CMP_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_2

HAL_StatusTypeDef VR_Init(uint32_t ckp_pulses_per_rev, uint32_t cmp_pulses_per_rev, uint32_t timeout_ms);

/**
 * @brief Executa a detecção de pulsos do CKP ou CMP
 * @param type Tipo do sensor
 */
void VR_InputCaptureCallback(VR_Sensor_Type_t type);

/**
 * @brief Retorna nome do sensor como string
 * @param type Tipo do sensor
 * @return String com nome do sensor
 */
const char* VR_GetSensorName(VR_Sensor_Type_t type);

#endif /* INC_MAX9924_DRIVER_H_ */
