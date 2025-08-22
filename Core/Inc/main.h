/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Sensor_OPT_1_OUT_Pin GPIO_PIN_2
#define Sensor_OPT_1_OUT_GPIO_Port GPIOE
#define Sensor_OPT_3_OUT_Pin GPIO_PIN_4
#define Sensor_OPT_3_OUT_GPIO_Port GPIOE
#define Sensor_OPT_4_OUT_Pin GPIO_PIN_5
#define Sensor_OPT_4_OUT_GPIO_Port GPIOE
#define Sensor_MAP_Pin GPIO_PIN_6
#define Sensor_MAP_GPIO_Port GPIOE
#define Bat_Voltage_Pin GPIO_PIN_0
#define Bat_Voltage_GPIO_Port GPIOC
#define Sensor_In_1_Pin GPIO_PIN_1
#define Sensor_In_1_GPIO_Port GPIOC
#define Sensor_In_2_Pin GPIO_PIN_2
#define Sensor_In_2_GPIO_Port GPIOC
#define Sensor_In_3_Pin GPIO_PIN_3
#define Sensor_In_3_GPIO_Port GPIOC
#define CKP_Pin GPIO_PIN_0
#define CKP_GPIO_Port GPIOA
#define CMP_Pin GPIO_PIN_1
#define CMP_GPIO_Port GPIOA
#define Sensor_In_6_Pin GPIO_PIN_2
#define Sensor_In_6_GPIO_Port GPIOA
#define Sensor_In_7_Pin GPIO_PIN_3
#define Sensor_In_7_GPIO_Port GPIOA
#define Sensor_In_8_Pin GPIO_PIN_4
#define Sensor_In_8_GPIO_Port GPIOA
#define SPI_CJ125_SCK_Pin GPIO_PIN_5
#define SPI_CJ125_SCK_GPIO_Port GPIOA
#define Sensor_In_9_Pin GPIO_PIN_6
#define Sensor_In_9_GPIO_Port GPIOA
#define Sensor_In_10_Pin GPIO_PIN_7
#define Sensor_In_10_GPIO_Port GPIOA
#define Sensor_In_4_Pin GPIO_PIN_4
#define Sensor_In_4_GPIO_Port GPIOC
#define Sensor_In_5_Pin GPIO_PIN_5
#define Sensor_In_5_GPIO_Port GPIOC
#define Sensor_In_11_Pin GPIO_PIN_0
#define Sensor_In_11_GPIO_Port GPIOB
#define Sensor_In_12_Pin GPIO_PIN_1
#define Sensor_In_12_GPIO_Port GPIOB
#define Injector_1_Pin GPIO_PIN_9
#define Injector_1_GPIO_Port GPIOE
#define Injector_2_Pin GPIO_PIN_11
#define Injector_2_GPIO_Port GPIOE
#define Injector_3_Pin GPIO_PIN_13
#define Injector_3_GPIO_Port GPIOE
#define Injector_4_Pin GPIO_PIN_14
#define Injector_4_GPIO_Port GPIOE
#define CMP_OUT_Pin GPIO_PIN_11
#define CMP_OUT_GPIO_Port GPIOD
#define Input_Ig_1_Pin GPIO_PIN_12
#define Input_Ig_1_GPIO_Port GPIOD
#define Input_Ig_2_Pin GPIO_PIN_13
#define Input_Ig_2_GPIO_Port GPIOD
#define Input_Ig_3_Pin GPIO_PIN_14
#define Input_Ig_3_GPIO_Port GPIOD
#define Input_Ig_4_Pin GPIO_PIN_15
#define Input_Ig_4_GPIO_Port GPIOD
#define PWM_MCU_OUT_1_Pin GPIO_PIN_6
#define PWM_MCU_OUT_1_GPIO_Port GPIOC
#define PWM_MCU_OUT_2_Pin GPIO_PIN_7
#define PWM_MCU_OUT_2_GPIO_Port GPIOC
#define SPI_CJ125_MOSI_Pin GPIO_PIN_7
#define SPI_CJ125_MOSI_GPIO_Port GPIOD
#define SPI_CJ125_MISO_Pin GPIO_PIN_4
#define SPI_CJ125_MISO_GPIO_Port GPIOB
#define PWM_MCU_OUT_3_Pin GPIO_PIN_5
#define PWM_MCU_OUT_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
