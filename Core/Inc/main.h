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
#define SPI_CJ125_SCK_Pin GPIO_PIN_5
#define SPI_CJ125_SCK_GPIO_Port GPIOA
#define CMP_OUT_Pin GPIO_PIN_11
#define CMP_OUT_GPIO_Port GPIOD
#define SPI_CJ125_MOSI_Pin GPIO_PIN_7
#define SPI_CJ125_MOSI_GPIO_Port GPIOD
#define SPI_CJ125_MISO_Pin GPIO_PIN_4
#define SPI_CJ125_MISO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
