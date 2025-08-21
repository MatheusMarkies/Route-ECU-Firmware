/*
 * engine_controll.c
 *
 *  Created on: Aug 19, 2025
 *      Author: Matheus Markies
 */

/*
 *Implementação Algorítmica do Speed-Density
 *O algoritmo Speed-Density é executado pela ECU a cada ciclo do motor para determinar a massa de combustível a ser injetada. O processo pode ser resumido nos seguintes passos:
 *Amostragem dos Sensores: A ECU lê os valores instantâneos dos sensores MAP, IAT e RPM.
 *Consulta à Tabela VE: Usando os valores atuais de MAP e RPM como coordenadas, a ECU consulta a tabela VE 2D. Se o ponto de operação exato (por exemplo, 2550 RPM e 85 kPa) não corresponder a um ponto de grade na tabela, a ECU utiliza um algoritmo de interpolação bilinear (detalhado na Seção 6.1) para calcular o valor preciso da VE entre os quatro pontos de grade circundantes.
 *Cálculo do Volume Efetivo (Vefetivo): O volume efetivo de ar admitido é calculado multiplicando-se o volume de deslocamento de um único cilindro pelo valor da VE obtido.
 *
 *Vefetivo = DeslocamentoCilindro * VE(MAP,RPM)
 *Cálculo da Massa de Ar (mar): Com a pressão (p do MAP), a temperatura (T do IAT, convertida para Kelvin)
 *e o volume efetivo (V efetivo), a ECU aplica a forma derivada da Lei dos Gases Ideais para calcular a massa de ar que preencheu o cilindro durante o ciclo de admissão.
 */

/*
 * engine_control.c
 *
 *  Created on: Aug 19, 2025
 *      Author: Matheus Markies
 */

//Nessa script falta varios valores do g_engineData
#include <math.h>
#include <string.h>
#include "engine_control.h"
#include "ecu_config.h"
#include "cmsis_os.h"

// --- Definição das Variáveis Globais ---
extern EcuConfig_t g_ecuConfig;
extern EngineData_t g_engineData;
extern EngineState_t g_engineState;

// Semáforo para sincronizar a tarefa com a ISR do CKP
osSemaphoreId engineSyncSemaphoreHandle;

// ----------------- UTILITÁRIOS DE TEMPO (baseado no RTOS) -----------------
static inline uint32_t now_us(void) {
	// Converte ticks do scheduler para micros (evita precisar de HAL_TIM quando não disponível)
	uint32_t ticks = osKernelSysTick();
	uint32_t hz = osKernelSysTickFrequency;  // ticks/seg
	if (hz == 0)
		hz = 1000;
	// cuidado com overflow: para cálculos de RPM usamos delta curto entre dentes
	return (uint32_t) ((((uint64_t) ticks) * 1000000ULL) / (uint64_t) hz);
}

// ----------------- CLAMPS E INTERPOLAÇÕES -----------------
static inline float clampf(float v, float lo, float hi) {
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Busca binária “lower index” (retorna i tal que axis[i] <= x < axis[i+1])
static int lower_index(const float *axis, int len, float x) {
	if (x <= axis[0])
		return 0;
	if (x >= axis[len - 2])
		return len - 2; // garante i+1 válido
	int lo = 0, hi = len - 1;
	while (hi - lo > 1) {
		int mid = (lo + hi) >> 1;
		if (x >= axis[mid])
			lo = mid;
		else
			hi = mid;
	}
	return lo;
}

// Interpolação 1D
static float interp1d(const float *x_axis, const float *y_vals, int len,
		float x) {
	if (len < 2)
		return y_vals[0];
	int i = lower_index(x_axis, len, x);
	float x0 = x_axis[i], x1 = x_axis[i + 1];
	float y0 = y_vals[i], y1 = y_vals[i + 1];
	float t = (x1 != x0) ? (x - x0) / (x1 - x0) : 0.0f;
	return y0 + t * (y1 - y0);
}

// ----------------- INTERPOLAÇÃO BILINEAR 2D -----------------
static float interpolate_bilinear(const Table2D_t *table, float rpm,
		float map_kpa) {
	const int nx = table->rows;
	const int ny = table->cols;

	// Clamp
	float xr = clampf(rpm, table->x_axis[0], table->x_axis[nx - 1]);
	float ym = clampf(map_kpa, table->y_axis[0], table->y_axis[ny - 1]);

	int i = lower_index(table->x_axis, nx, xr);
	int j = lower_index(table->y_axis, ny, ym);

	float x0 = table->x_axis[i], x1 = table->x_axis[i + 1];
	float y0 = table->y_axis[j], y1 = table->y_axis[j + 1];

	// acesso linear: [i * cols + j]
	float q11 = table->values[i * ny + j];
	float q21 = table->values[(i + 1) * ny + j];
	float q12 = table->values[i * ny + (j + 1)];
	float q22 = table->values[(i + 1) * ny + (j + 1)];

	float fx = (x1 != x0) ? (xr - x0) / (x1 - x0) : 0.0f;
	float fy = (y1 != y0) ? (ym - y0) / (y1 - y0) : 0.0f;

	float v1 = q11 * (1.0f - fx) + q21 * fx;
	float v2 = q12 * (1.0f - fx) + q22 * fx;
	return v1 * (1.0f - fy) + v2 * fy;
}

// ----------------- CÁLCULO MASSA DE AR (SD) -----------------
static float calculate_air_mass_sd(void) {
	// Mistura de temperatura de carga (opcional): aqui usamos IAT pura para simplificar
	float charge_temp_k = g_engineData.iat_celsius + 273.15f;

	float ve_percent = interpolate_bilinear(&g_ecuConfig.table_ve,
			g_engineState.rpm, g_engineData.map_kpa);

	float v_cyl_l = g_ecuConfig.engine_displacement_l
			/ (float) g_ecuConfig.num_cylinders;

	// Usar R em kPa*L/(g*K) = 0.287 (equivalente a 287 J/(kg*K))
	// AirMass_g = (MAP_kPa * V_cyl_L * VE) / (R_kPaL_gK * T_K)
	const float R_KPAL_PER_GK = 0.287f;
	float air_mass_g = (g_engineData.map_kpa * v_cyl_l * (ve_percent * 0.01f))
			/ (R_KPAL_PER_GK * charge_temp_k);

	return air_mass_g;
}

// ----------------- ENRIQUECIMENTOS (Gamma) -----------------
static float calculate_enrichments_multipliers(void) {
	// Warm-Up Enrichment (WUE) via ECT -> multiplicador
	//float wue = interp1d(g_ecuConfig.wue.axis_x, g_ecuConfig.wue.values, g_ecuConfig.wue.len, g_engineData.ect_celsius);

	// Accel Enrichment (AE) via |TPSdot| (ou MAPdot se preferir)
	float tpsdot = g_engineData.tps_percent; // %/s (garanta que você atualiza isso no outro módulo)
	float ae = 1.0f;
	ae = interp1d(g_ecuConfig.table_accel_enrichment.x_axis,
			g_ecuConfig.table_accel_enrichment.values,
			g_ecuConfig.table_accel_enrichment.len, fabsf(tpsdot));

	// Correção IAT direta (se desejar além do uso na massa de ar)
	//float iat_corr = 1.0f;
	//iat_corr = interp1d(g_ecuConfig.iat_corr.axis_x, g_ecuConfig.iat_corr.values, g_ecuConfig.iat_corr.len, g_engineData.iat_celsius);

	// Baro factor (MAP/BaroRef) — útil para altitude
	//float baro_ref = (g_engineState.baro_kpa > 10.0f) ? g_engineState.baro_kpa : g_ecuConfig.baro_ref_kpa;
	//float baro_factor = (baro_ref > 1.0f) ? (g_engineData.map_kpa / baro_ref) : 1.0f;

	float gamma = ae;    //wue * ae * iat_corr * baro_factor;
	return clampf(gamma, g_ecuConfig.gamma_min, g_ecuConfig.gamma_max);
}

// ----------------- CLOSED LOOP O2 (STFT/LTFT com PI) -----------------
typedef struct {
	float kp;
	float ki;
	float stft;    // curto prazo (fração, e.g. +0.03 = +3%)
	float ltft;    // longo prazo (fração)
} O2Ctrl_t;

static O2Ctrl_t g_o2 = { 0 };

static inline int closed_loop_enabled(void) {
	// Só fecha malha quando motor quente e dentro de faixa de carga/rotação
	if (g_engineData.ect_celsius < g_ecuConfig.clo.min_ect_c)
		return 0;
	if (g_engineState.rpm < g_ecuConfig.clo.min_rpm
			|| g_engineState.rpm > g_ecuConfig.clo.max_rpm)
		return 0;
	if (g_engineData.map_kpa < g_ecuConfig.clo.min_map_kpa
			|| g_engineData.map_kpa > g_ecuConfig.clo.max_map_kpa)
		return 0;
	return 1;
}

static void o2_controller_init(void) {
	g_ecuConfig.clo.kp = g_ecuConfig.clo.kp;
	g_ecuConfig.clo.ki = g_ecuConfig.clo.ki;
	g_ecuConfig.clo.stft = 0.0f;
	g_ecuConfig.clo.ltft = 0.0f;
}

static void o2_controller_update(float afr_measured, float afr_target,
		float dt_s) {
	float err =
			(afr_target > 0.1f) ?
					((afr_target - afr_measured) / afr_target) : 0.0f; // erro relativo
	// STFT = PI curto prazo
	g_ecuConfig.clo.stft += (g_o2.kp * err);
	g_ecuConfig.clo.stft = clampf(g_ecuConfig.clo.stft,
			-g_ecuConfig.clo.stft_limit, g_ecuConfig.clo.stft_limit);

	// LTFT integra lento somente se erro pequeno e estável
	float integ_condition = fabsf(err) < g_ecuConfig.clo.ltft_err_gate;
	if (integ_condition) {
		g_ecuConfig.clo.ltft += (g_o2.ki * err * dt_s);
		g_ecuConfig.clo.ltft = clampf(g_ecuConfig.clo.ltft,
				-g_ecuConfig.clo.ltft_limit, g_ecuConfig.clo.ltft_limit);
	} else {
		// anti-windup leve
		g_ecuConfig.clo.ltft *= 0.999f;
	}
}

// ----------------- DEADTIME (1D por Vbat) -----------------
static float injector_deadtime_ms(float vbat) {
	if (g_ecuConfig.table_injector_deadtime.len <= 0)
		return 1.0f;
	return interp1d(g_ecuConfig.table_injector_deadtime.x_axis,
			g_ecuConfig.table_injector_deadtime.values,
			g_ecuConfig.table_injector_deadtime.len, vbat);
}

// ----------------- DFCO -----------------
static int DFCO_active_conditions(void) {
	if (g_ecuConfig.num_cylinders > 4)
		return 0;
	if (g_engineData.tps_percent > g_ecuConfig.dfco.max_tps_percent)
		return 0;
	if (g_engineData.map_kpa > g_ecuConfig.dfco.max_map_kpa)
		return 0;
	if (g_engineState.rpm < g_ecuConfig.dfco.min_rpm)
		return 0;
	if (g_engineData.ect_celsius < g_ecuConfig.dfco.min_ect_c)
		return 0;
	return 1;
}

void EngineControl_Init(void) {
	osSemaphoreDef(engineSync);
	engineSyncSemaphoreHandle = osSemaphoreCreate(osSemaphore(engineSync), 1);
	osSemaphoreWait(engineSyncSemaphoreHandle, 0);

	// Baro inicial (KOEO) – se tiver rotina específica, chame-a no boot e salve em g_engineState.baro_kpa
	if (g_engineState.baro_kpa < 10.0f)
		g_engineState.baro_kpa = g_ecuConfig.baro_ref_kpa;

	o2_controller_init();
}

void EngineControl_RunCalculations(void) {
	// 1. Massa de ar por cilindro (SD)
	g_engineState.air_mass_per_cyl_g = calculate_air_mass_sd();

	// 2. AFR alvo (tabela 2D RPM x MAP)
	float afr_target = interpolate_bilinear(&g_ecuConfig.table_afr_target,
			g_engineState.rpm, g_engineData.map_kpa);
	afr_target = clampf(afr_target, g_ecuConfig.afr_min, g_ecuConfig.afr_max);

	// 3. Massa de combustível por cilindro
	float fuel_mass_g = g_engineState.air_mass_per_cyl_g / afr_target;

	// 4. PW base (s)
	float inj_flow_gps = fmaxf(g_ecuConfig.injector_flow_gps, 0.0001f);
	float pw_base_s = fuel_mass_g / inj_flow_gps;

	// 5. Enriquecimentos (Gamma)
	float gamma = calculate_enrichments_multipliers();
	float pw_enriched_s = pw_base_s * gamma;

	// 6. Closed loop O2 (STFT/LTFT)
	float pw_closedloop_s = pw_enriched_s;
	static uint32_t last_us = 0;
	uint32_t now = now_us();
	float dt_s =
			(last_us == 0) ? (1.0f / 1000.0f) : ((now - last_us) / 1000000.0f);
	last_us = now;

	if (closed_loop_enabled() && g_engineData.lambda > 0.1f) {
		float afr_meas = g_engineData.lambda * g_ecuConfig.afr_stoich;
		o2_controller_update(afr_meas, afr_target, dt_s);
		float clo_gain = 1.0f + g_o2.stft + g_o2.ltft;     // fração
		clo_gain = clampf(clo_gain, g_ecuConfig.clo.min_gain,
				g_ecuConfig.clo.max_gain);
		pw_closedloop_s *= clo_gain;
	}

	// 7. Deadtime (ms -> s)
	float deadtime_ms = injector_deadtime_ms(g_engineData.battery_voltage);
	float pw_final_s = pw_closedloop_s + (deadtime_ms * 0.001f);

	// 8. DFCO
	if (DFCO_active_conditions() == 1) {
		//pw_final_s = 0.0f;

		// Limites de segurança
		pw_final_s = clampf(pw_final_s, g_ecuConfig.pw_min_s,
				g_ecuConfig.pw_max_s);

		g_engineState.final_fuel_pw_ms = pw_final_s * 1000.0f;

		// 10. Agendar Injeção (módulo de atuadores)
		Injector_SchedulePulse(g_engineState.current_cylinder,
				g_engineState.final_fuel_pw_ms);

		// 11. Ignição (exemplo: buscar avanço e agendar)
		float ign_deg = interpolate_bilinear(&g_ecuConfig.table_ign_advance,
				g_engineState.rpm, g_engineData.map_kpa);
		ign_deg = clampf(ign_deg, g_ecuConfig.ign_min_deg,
				g_ecuConfig.ign_max_deg);
		float dwell_ms = 3.2f;
		Ignition_ScheduleSpark(g_engineState.current_cylinder, ign_deg,
				dwell_ms);
	}
}

// Esta é a função que a Interrupção do CKP (ISR) irá chamar
void EngineControl_CrankEventCallback(void) {
	// **RÁPIDA**: calcule RPM/ângulo e acorde a tarefa

	static uint32_t last_edge_us = 0;
	uint32_t t_us = now_us();

	// Cálculo de RPM usando tempo entre dentes
	if (last_edge_us != 0) {
		uint32_t dt_us = t_us - last_edge_us;
		if (dt_us > 0) {
			// dentes úteis por volta (excluindo falhas/ausências): defina em g_ecuConfig.ckp.teeth_per_rev
			float teeth_per_rev = (float) g_ecuConfig.ckp.teeth_per_rev;
			float rev_per_s = (1000000.0f / (float) dt_us) / teeth_per_rev;
			g_engineState.rpm = (uint16_t) clampf(60.0f * rev_per_s, 0.0f,
					(float) g_ecuConfig.rpm_max);
		}
	}
	last_edge_us = t_us;

	// Atualiza ângulo do motor (incremento por dente)
	float deg_per_tooth = 360.0f / (float) g_ecuConfig.ckp.teeth_per_rev;
	g_engineState.engine_angle_deg += deg_per_tooth;
	if (g_engineState.engine_angle_deg >= 720.0f) { // 4 tempos
		g_engineState.engine_angle_deg -= 720.0f;
		// avance cilindro corrente (sequencial)
		g_engineState.current_cylinder = (g_engineState.current_cylinder
				% g_ecuConfig.num_cylinders) + 1;
	}

	// 2. Acorda a tarefa principal para cálculos fora da ISR
	osSemaphoreRelease(engineSyncSemaphoreHandle);
}

// A tarefa do RTOS que orquestra o controlo
void Task_EngineControl(void const *argument) {
	EngineControl_Init();

	for (;;) {
		if (osSemaphoreWait(engineSyncSemaphoreHandle, osWaitForever) == osOK) {
			EngineControl_RunCalculations();
		}
	}
}
