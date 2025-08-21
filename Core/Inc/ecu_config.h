/*
 * ecu_config.h
 *
 * Created on: 21 de ago de 2025
 * Author: Matheus Markies
 *
 * Estrutura de configuração central e tipos de dados para a ECU.
 * Define todos os parâmetros e tabelas de calibração.
 */

#ifndef INC_ECU_CONFIG_H_
#define INC_ECU_CONFIG_H_

#include <stdint.h>

// --- TIPOS DE DADOS BÁSICOS ---

typedef enum {
    SENSOR_TYPE_NONE = 0, SENSOR_TYPE_TPS, SENSOR_TYPE_CLT, SENSOR_TYPE_IAT,
    SENSOR_TYPE_MAP, SENSOR_TYPE_O2, SENSOR_TYPE_CKP, SENSOR_TYPE_CMP
} SensorType_e;

typedef enum {
    SENSOR_INPUT_VR = 0, SENSOR_INPUT_HALL = 1
} PositionSensorType_e;

// --- ESTRUTURAS PARA TABELAS DE CALIBRAÇÃO ---

typedef struct {
    int len;             // Número de pontos na tabela
    const float *x_axis; // Ponteiro para o eixo X (ex: Temperatura)
    const float *values; // Ponteiro para os valores (ex: Multiplicador)
} Table1D_t;

typedef struct {
    int rows;            // Número de linhas (eixo X)
    int cols;            // Número de colunas (eixo Y)
    const float *x_axis; // Ponteiro para o eixo X (ex: RPM)
    const float *y_axis; // Ponteiro para o eixo Y (ex: MAP)
    const float *values; // Ponteiro para a matriz de valores (acesso linear: [i * cols + j])
} Table2D_t;

// --- SUB-ESTRUTURAS DE CONFIGURAÇÃO ---

typedef struct {
    uint16_t teeth_per_rev; // Dentes por rotação (sem contar a falha, ex: 60)
    float    tdc_angle_after_gap_deg; // Ângulo do PMS após a falha
    int num_teeth_on_wheel;
} TriggerWheelConfig_t;

typedef struct {
    float kp;
    float ki;
    float min_ect_c;
    uint16_t min_rpm;
    uint16_t max_rpm;
    float min_map_kpa;
    float max_map_kpa;
    float stft_limit;  // Limite para Short Term Fuel Trim (ex: 0.25 para +/- 25%)
    float ltft_limit;  // Limite para Long Term Fuel Trim
    float ltft_err_gate; // Limite de erro para permitir a integração do LTFT

    float stft;
    float ltft;

    float min_gain;
    float max_gain;
} ClosedLoopO2Config_t;

typedef struct {
    float max_tps_percent;
    float max_map_kpa;
    uint16_t min_rpm;
    float min_ect_c;
} DfcoConfig_t;


// --- ESTRUTURA DE CONFIGURAÇÃO PRINCIPAL ---

typedef struct {
    uint32_t config_version;

    // --- Configuração do Motor e Injetores ---
    uint8_t num_cylinders;
    float engine_displacement_l;
    float injector_flow_gps;
    float afr_stoich;
    float required_fuel_const;
    uint16_t rpm_max;

    // --- Mapeamento de Sensores ---
    SensorType_e sensor_channel_map[12];
    PositionSensorType_e ckp_type;
    TriggerWheelConfig_t ckp;

    // --- Limites e Failsafes ---
    float afr_min, afr_max;
    float pw_min_s, pw_max_s;
    float ign_min_deg, ign_max_deg;
    float baro_ref_kpa; // Pressão barométrica de referência
    float gamma_min, gamma_max;

    // --- Módulos de Controlo ---
    ClosedLoopO2Config_t clo; // Configuração para Malha Fechada (O2)
    DfcoConfig_t         dfco; // Configuração para Corte de Combustível

    // --- Tabelas de Calibração ---
    Table2D_t table_ve;
    Table2D_t table_afr_target;
    Table2D_t table_ign_advance; // Renomeado de 'spark_advance' para consistência
    Table1D_t table_warmup_enrichment;
    Table1D_t table_accel_enrichment;
    Table1D_t table_injector_deadtime; // Renomeado para consistência

} EcuConfig_t;


// --- Estruturas de Dados em Tempo Real (Não fazem parte da config) ---
typedef struct {
    float tps_percent;
    float ect_celsius;
    float iat_celsius;
    float map_kpa;
    float battery_voltage;
    float lambda;
} EngineData_t;

typedef struct {
    uint32_t rpm;
    float    engine_angle_deg;
    uint8_t  current_cylinder;
    float    air_mass_per_cyl_g;
    float    final_fuel_pw_ms;
    float    final_spark_deg;
    int      is_in_dfco;
    float    baro_kpa; // Barômetro medido no arranque (KOEO)
} EngineState_t;


#endif /* INC_ECU_CONFIG_H_ */
