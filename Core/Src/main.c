/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "ADC_driver.h"
#include "cJSON.h"
#include "communication_serial_protocol.h"
#include "MAX9924_driver.h"
#include "engine_control.h"
#include "Battery_manager.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define __HAL_TIM_SET_OC_MODE(__HANDLE__, __CHANNEL__, __MODE__) \
do { \
    if ((__CHANNEL__) == TIM_CHANNEL_1) { \
        MODIFY_REG((__HANDLE__)->Instance->CCMR1, TIM_CCMR1_OC1M, (__MODE__)); \
    } else if ((__CHANNEL__) == TIM_CHANNEL_2) { \
        MODIFY_REG((__HANDLE__)->Instance->CCMR1, TIM_CCMR1_OC2M, ((__MODE__) << 8U)); \
    } else if ((__CHANNEL__) == TIM_CHANNEL_3) { \
        MODIFY_REG((__HANDLE__)->Instance->CCMR2, TIM_CCMR2_OC3M, (__MODE__)); \
    } else if ((__CHANNEL__) == TIM_CHANNEL_4) { \
        MODIFY_REG((__HANDLE__)->Instance->CCMR2, TIM_CCMR2_OC4M, ((__MODE__) << 8U)); \
    } \
} while(0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void I2C_Scanner(I2C_HandleTypeDef *hi2c);
char* ADC_GenerateJSON(void);
char* VR_GenerateJSON(void);
void ADC_Read_Cycle(void);

void TIM1_Init_Config(void);
void TIM4_Init_Config(void);
void TIM5_Init_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t captureValue = 0;
uint32_t previousCaptureValue = 0;
uint32_t frequency = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM5 && htim->Channel == CKP_ACTIVE_CHANNEL) {
		VR_InputCaptureCallback(SENSOR_CKP);
	}
	if (htim->Instance == TIM5 && htim->Channel == CMP_ACTIVE_CHANNEL) {
		VR_InputCaptureCallback(SENSOR_CMP);
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		uint32_t current_time = __HAL_TIM_GET_COUNTER(htim);

		ENGINE_VR_OutputCompareCallback(current_time, ckp_sensor, cmp_sensor);

		uint32_t next_compare = current_time + 100;//100us
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, next_compare);
	}

	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_5) {
		uint32_t current_time = __HAL_TIM_GET_COUNTER(htim);

		ENGINE_Injector_OutputCompareCallback(current_time);

		uint32_t next_compare = current_time + 100;
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_5, next_compare);
	}

	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_6) {
		uint32_t current_time = __HAL_TIM_GET_COUNTER(htim);

		ENGINE_Ignition_OutputCompareCallback(current_time);

		uint32_t next_compare = current_time + 100;
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_6, next_compare);
	}

	if (htim->Instance == TIM1) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			if (injector_state[0].state == INJ_STATE_SCHEDULED) {
				uint32_t scheduled_end = injector_state[0].pulse_start_time
						+ injector_state[0].pulse_width_us;

				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, scheduled_end);
				__HAL_TIM_SET_OC_MODE(htim, TIM_CHANNEL_1, TIM_OCMODE_INACTIVE);

				injector_state[0].state = INJ_STATE_ACTIVE;
			} else {
				injector_state[0].state = INJ_STATE_IDLE;
				__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			if (injector_state[1].state == INJ_STATE_SCHEDULED) {
				uint32_t scheduled_end = injector_state[1].pulse_start_time
						+ injector_state[1].pulse_width_us;

				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, scheduled_end);
				__HAL_TIM_SET_OC_MODE(htim, TIM_CHANNEL_2, TIM_OCMODE_INACTIVE);

				injector_state[1].state = INJ_STATE_ACTIVE;
			} else {
				injector_state[1].state = INJ_STATE_IDLE;
				__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			if (injector_state[2].state == INJ_STATE_SCHEDULED) {
				uint32_t scheduled_end = injector_state[2].pulse_start_time
						+ injector_state[2].pulse_width_us;

				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, scheduled_end);
				__HAL_TIM_SET_OC_MODE(htim, TIM_CHANNEL_3, TIM_OCMODE_INACTIVE);

				injector_state[2].state = INJ_STATE_ACTIVE;
			} else {
				injector_state[2].state = INJ_STATE_IDLE;
				__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC3);
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			if (injector_state[3].state == INJ_STATE_SCHEDULED) {
				uint32_t scheduled_end = injector_state[3].pulse_start_time
						+ injector_state[3].pulse_width_us;

				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, scheduled_end);
				__HAL_TIM_SET_OC_MODE(htim, TIM_CHANNEL_4, TIM_OCMODE_INACTIVE);


				injector_state[3].state = INJ_STATE_ACTIVE;
			} else {
				injector_state[3].state = INJ_STATE_IDLE;
				__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC4);
			}
		}
	}

	if (htim->Instance == TIM4) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			if (ignition_state[0].state == IGN_STATE_DWELL) {

				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1,
						ignition_state[0].spark_time_us);
				__HAL_TIM_SET_OC_MODE(htim, TIM_CHANNEL_1, TIM_OCMODE_INACTIVE);

				ignition_state[0].state = IGN_STATE_SPARK;
			} else if (ignition_state[0].state == IGN_STATE_SPARK) {
				ignition_state[0].state = IGN_STATE_IDLE;
				__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			if (ignition_state[1].state == IGN_STATE_DWELL) {

				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2,
						ignition_state[1].spark_time_us);
				__HAL_TIM_SET_OC_MODE(htim, TIM_CHANNEL_2, TIM_OCMODE_INACTIVE);

				ignition_state[1].state = IGN_STATE_SPARK;
			} else if (ignition_state[1].state == IGN_STATE_SPARK) {
				ignition_state[1].state = IGN_STATE_IDLE;
				__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			if (ignition_state[2].state == IGN_STATE_DWELL) {

				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3,
						ignition_state[2].spark_time_us);
				__HAL_TIM_SET_OC_MODE(htim, TIM_CHANNEL_3, TIM_OCMODE_INACTIVE);

				ignition_state[2].state = IGN_STATE_SPARK;
			} else if (ignition_state[2].state == IGN_STATE_SPARK) {
				ignition_state[2].state = IGN_STATE_IDLE;
				__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC3);
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			if (ignition_state[3].state == IGN_STATE_DWELL) {

				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4,
						ignition_state[3].spark_time_us);
				__HAL_TIM_SET_OC_MODE(htim, TIM_CHANNEL_4, TIM_OCMODE_INACTIVE);

				ignition_state[3].state = IGN_STATE_SPARK;
			} else if (ignition_state[3].state == IGN_STATE_SPARK) {
				ignition_state[3].state = IGN_STATE_IDLE;
				__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC4);
			}
		}
	}
}

void I2C_Scanner(I2C_HandleTypeDef *hi2c) {
	char smallBuf[16];
	uint8_t devices_found = 0;

	printf("\r\n--- Scanning I2C bus ---\r\n");

	for (uint8_t addr = 1; addr < 128; addr++) {
		if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t) (addr << 1), 2, 10)
				== HAL_OK) {

			snprintf(smallBuf, sizeof(smallBuf), "0x%02X", addr);
			printf("Device found at: %s (8-bit: 0x%02X)\r\n", smallBuf,
					addr << 1);

			devices_found++;
		}
	}

	if (devices_found == 0) {
		printf("No I2C devices found!\r\n");
	} else {
		printf("\r\nTotal devices found: %u\r\n", devices_found);
	}

	printf("--- Scan Complete ---\r\n");
}

char* ADC_GenerateJSON(void) {
	cJSON *root = cJSON_CreateObject();

	if (root == NULL) {
		return NULL;
	}

	cJSON *adc_u16_array = cJSON_CreateArray();
	for (int i = 0; i < 8; i++) {
		cJSON *sensor = cJSON_CreateObject();
		cJSON_AddStringToObject(sensor, "sensor", AD7998_U16_GetSensorName(i));
		cJSON_AddNumberToObject(sensor, "raw", adc_u16.raw_values[i]);
		cJSON_AddNumberToObject(sensor, "voltage", adc_u16.voltages[i]);
		cJSON_AddItemToArray(adc_u16_array, sensor);
	}
	cJSON_AddItemToObject(root, "adc_u16", adc_u16_array);

	cJSON *adc_u17_array = cJSON_CreateArray();
	for (int i = 0; i < 8; i++) {
		cJSON *sensor = cJSON_CreateObject();
		cJSON_AddStringToObject(sensor, "sensor", AD7998_U17_GetSensorName(i));
		cJSON_AddNumberToObject(sensor, "raw", adc_u17.raw_values[i]);
		cJSON_AddNumberToObject(sensor, "voltage", adc_u17.voltages[i]);
		cJSON_AddItemToArray(adc_u17_array, sensor);
	}
	cJSON_AddItemToObject(root, "adc_u17", adc_u17_array);

	char *json_string = cJSON_PrintUnformatted(root);

	if (json_string != NULL) {
		size_t total_len = strlen(json_string) + 1;
		char *final_string = (char*) malloc(total_len);
		if (final_string != NULL) {
			snprintf(final_string, total_len, "%s", json_string);
		}
		cJSON_free(json_string);
		cJSON_Delete(root);
		return final_string;
	}

	cJSON_Delete(root);
	return NULL;
}

char* VR_GenerateJSON(void) {
	cJSON *root = cJSON_CreateObject();
	if (root == NULL) {
		return NULL;
	}

	cJSON *ckp = cJSON_CreateObject();
	cJSON *cmp = cJSON_CreateObject();
	if (ckp == NULL || cmp == NULL) {
		cJSON_Delete(root);
		return NULL;
	}

	cJSON_AddNumberToObject(ckp, "rpm", ckp_sensor.rpm);
	cJSON_AddNumberToObject(ckp, "freq", ckp_sensor.frequency_hz);
	cJSON_AddNumberToObject(ckp, "pulses", ckp_sensor.pulse_count);
	cJSON_AddNumberToObject(ckp, "revolutions", ckp_sensor.revolution_count);
	cJSON_AddNumberToObject(ckp, "period", ckp_sensor.period);

	cJSON_AddNumberToObject(cmp, "rpm", cmp_sensor.rpm);
	cJSON_AddNumberToObject(cmp, "freq", cmp_sensor.frequency_hz);
	cJSON_AddNumberToObject(cmp, "pulses", cmp_sensor.pulse_count);
	cJSON_AddNumberToObject(cmp, "revolutions", cmp_sensor.revolution_count);
	cJSON_AddNumberToObject(cmp, "period", cmp_sensor.period);

	cJSON_AddItemToObject(root, "ckp", ckp);
	cJSON_AddItemToObject(root, "cmp", cmp);

	char *json_string = cJSON_PrintUnformatted(root);

	if (json_string != NULL) {
		size_t total_len = strlen(json_string) + 1;
		char *final_string = (char*) malloc(total_len);
		if (final_string != NULL) {
			snprintf(final_string, total_len, "%s", json_string);
		}
		cJSON_free(json_string);
		cJSON_Delete(root);
		return final_string;
	}

	cJSON_Delete(root);
	return NULL;
}

char* BATTERY_GenerateJSON(void) {
	cJSON *root = cJSON_CreateObject();
	if (root == NULL) {
		return NULL;
	}

	cJSON *battery_object = cJSON_CreateObject();
	if (battery_object == NULL) {
		cJSON_Delete(root);
		return NULL;
	}

	char str_buffer[16];
	snprintf(str_buffer, sizeof(str_buffer), "%.3f", battery.voltage);
	cJSON_AddRawToObject(battery_object, "voltage", str_buffer);

	cJSON_AddItemToObject(root, "battery", battery_object);

	char *json_string = cJSON_PrintUnformatted(root);

	if (json_string != NULL) {
		size_t total_len = strlen(json_string) + 1;
		char *final_string = (char*) malloc(total_len);
		if (final_string != NULL) {
			snprintf(final_string, total_len, "%s", json_string);
		}
		cJSON_free(json_string);
		cJSON_Delete(root);
		return final_string;
	}

	cJSON_Delete(root);
	return NULL;
}

void SERIAL_VrJSONCallback(Command_Result_t result, char *response) {
	if (result == CMD_RESULT_SUCCESS) {
		char *vr_json = VR_GenerateJSON();
		if (vr_json != NULL) {
			SERIAL_SendCommand(vr_json, "OK", 200, NULL);

			free(vr_json);
		} else {
			printf("Error generating JSON from VR sensors\r\n");
		}
	} else {
		return;
	}
}

void SERIAL_ADCJSONCallback(Command_Result_t result, char *response) {
	if (result == CMD_RESULT_SUCCESS) {
		char *adc_json = ADC_GenerateJSON();
		if (adc_json != NULL) {
			SERIAL_SendCommand(adc_json, "OK", 200, NULL);

			free(adc_json);
		} else {
			printf("Error generating JSON from ADCs sensors\r\n");
		}
	} else {
		return;
	}
}

void SERIAL_BatteryJSONCallback(Command_Result_t result, char *response) {
	if (result == CMD_RESULT_SUCCESS) {
		char *battery_json = BATTERY_GenerateJSON();
		if (battery_json != NULL) {
			SERIAL_SendCommand(battery_json, "OK", 200, NULL);

			free(battery_json);
		} else {
			printf("Error generating JSON from Battery Manager\r\n");
		}
	} else {
		return;
	}
}

void ADC_Read_Cycle(void) {
	if (AD7998_U16_ReadAllChannels() != HAL_OK) {
		printf("Erro ao ler ADC U16\r\n");
		return;
	}

	if (AD7998_U17_ReadAllChannels() != HAL_OK) {
		printf("Erro ao ler ADC U17\r\n");
		return;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	MX_TIM5_Init();
	MX_TIM4_Init();
	MX_TIM1_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	I2C_Scanner(&hi2c1);
	I2C_Scanner(&hi2c2);

	TIM1_Init_Config();
	TIM4_Init_Config();
	TIM5_Init_Config();

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	//Protocolo de Software Desktop
	huart_instance = &huart1;
	HAL_UART_Receive_IT(&huart1, &PROTOCOL_RX_Stream_Data, 1);

	SERIAL_ResetBuffers();
	SERIAL_SendCommand("AT", "OK", 1000, SERIAL_CheckConnection);

	//Inicializacão dos ADCs
	if (AD7998_Init(&hi2c2, 3.3f) != HAL_OK) {
		printf("Failed to initialize ADCs!\r\n");
	}

	if (VR_Init(58, 1, 1000) == HAL_OK) {
		printf("VR sensors successfully initialized!\r\n");
	} else
		printf("Error starting VR sensors!\r\n");

	printf("\r\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		static uint32_t last_ADC_read = 0;
		if (HAL_GetTick() - last_ADC_read >= 5) {
			last_ADC_read = HAL_GetTick();
			ADC_Read_Cycle();
		}

		static uint32_t last_BATTERY_read = 0;
		if (HAL_GetTick() - last_BATTERY_read >= 15) {
			last_BATTERY_read = HAL_GetTick();
			BATTERY_ReadVoltageFiltered();
		}

		SERIAL_CheckRXCommand();

		if (is_connected) {
			static uint32_t last_vr_send = 0;
			if (HAL_GetTick() - last_vr_send >= 1000) {
				last_vr_send = HAL_GetTick();

				SERIAL_SendCommand("AT+VR", "OK", 50, SERIAL_VrJSONCallback);
			}

			static uint32_t last_ADC_send = 0;
			if (HAL_GetTick() - last_ADC_send >= 1150) {
				last_ADC_send = HAL_GetTick();

				SERIAL_SendCommand("AT+ADC", "OK", 50, SERIAL_ADCJSONCallback);
			}

			static uint32_t last_BATTERY_send = 0;
			if (HAL_GetTick() - last_BATTERY_send >= 1250) {
				last_BATTERY_send = HAL_GetTick();

				SERIAL_SendCommand("AT+BATTERY", "OK", 50,
						SERIAL_BatteryJSONCallback);
			}
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV2;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 2;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_16B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.Oversampling.Ratio = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00909BEB;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00909BEB;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 79;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_6) != HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 79;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 79;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 0xFFFFFFFF;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void TIM1_Init_Config(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5,
			__HAL_TIM_GET_COUNTER(&htim1) + 10); //__HAL_TIM_GET_COUNTER(&htim1) + 10 a cada 10 ticks
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_5);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_6,
			__HAL_TIM_GET_COUNTER(&htim1) + 10); //__HAL_TIM_GET_COUNTER(&htim1) + 10 a cada 10 ticks
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_6);

	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_Base_Start(&htim1);
}

void TIM4_Init_Config(void) {
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);

	HAL_TIM_Base_Start(&htim4);
}

void TIM5_Init_Config(void) {
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);

	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3,
			__HAL_TIM_GET_COUNTER(&htim5) + 10); //__HAL_TIM_GET_COUNTER(&htim5) + 10 a cada 10 ticks
	HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_3);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		PROTOCOL_RX_Callback();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		PROTOCOL_TX_Callback();
	}
}

int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart3, (uint8_t*) ptr, len, 1000);
	return len;
}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
