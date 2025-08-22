/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h7xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                        /**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED=0;

/**
  * @brief ADC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC1)
  {
    /* USER CODE BEGIN ADC1_MspInit 0 */

    /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_INP10
    PC1     ------> ADC1_INP11
    PA2     ------> ADC1_INP14
    PA3     ------> ADC1_INP15
    PA4     ------> ADC1_INP18
    PA6     ------> ADC1_INP3
    PA7     ------> ADC1_INP7
    PC4     ------> ADC1_INP4
    PC5     ------> ADC1_INP8
    PB0     ------> ADC1_INN5
    PB0     ------> ADC1_INP9
    PB1     ------> ADC1_INP5
    */
    GPIO_InitStruct.Pin = Bat_Voltage_Pin|Sensor_In_1_Pin|Sensor_In_4_Pin|Sensor_In_5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Sensor_In_6_Pin|Sensor_In_7_Pin|Sensor_In_8_Pin|Sensor_In_9_Pin
                          |Sensor_In_10_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Sensor_In_11_Pin|Sensor_In_12_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */
  }
  else if(hadc->Instance==ADC2)
  {
    /* USER CODE BEGIN ADC2_MspInit 0 */

    /* USER CODE END ADC2_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PC0     ------> ADC2_INP10
    PC1     ------> ADC2_INP11
    PA2     ------> ADC2_INP14
    PA3     ------> ADC2_INP15
    PA4     ------> ADC2_INP18
    PA6     ------> ADC2_INP3
    PA7     ------> ADC2_INP7
    PC4     ------> ADC2_INP4
    PC5     ------> ADC2_INP8
    */
    GPIO_InitStruct.Pin = Bat_Voltage_Pin|Sensor_In_1_Pin|Sensor_In_4_Pin|Sensor_In_5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Sensor_In_6_Pin|Sensor_In_7_Pin|Sensor_In_8_Pin|Sensor_In_9_Pin
                          |Sensor_In_10_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC2_MspInit 1 */

    /* USER CODE END ADC2_MspInit 1 */
  }
  else if(hadc->Instance==ADC3)
  {
    /* USER CODE BEGIN ADC3_MspInit 0 */

    /* USER CODE END ADC3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC3 GPIO Configuration
    PC0     ------> ADC3_INP10
    PC1     ------> ADC3_INP11
    PC2_C     ------> ADC3_INP0
    PC3_C     ------> ADC3_INP1
    */
    GPIO_InitStruct.Pin = Bat_Voltage_Pin|Sensor_In_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_OPEN);

    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_OPEN);

    /* USER CODE BEGIN ADC3_MspInit 1 */

    /* USER CODE END ADC3_MspInit 1 */
  }

}

/**
  * @brief ADC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_INP10
    PC1     ------> ADC1_INP11
    PA2     ------> ADC1_INP14
    PA3     ------> ADC1_INP15
    PA4     ------> ADC1_INP18
    PA6     ------> ADC1_INP3
    PA7     ------> ADC1_INP7
    PC4     ------> ADC1_INP4
    PC5     ------> ADC1_INP8
    PB0     ------> ADC1_INN5
    PB0     ------> ADC1_INP9
    PB1     ------> ADC1_INP5
    */
    HAL_GPIO_DeInit(GPIOC, Bat_Voltage_Pin|Sensor_In_1_Pin|Sensor_In_4_Pin|Sensor_In_5_Pin);

    HAL_GPIO_DeInit(GPIOA, Sensor_In_6_Pin|Sensor_In_7_Pin|Sensor_In_8_Pin|Sensor_In_9_Pin
                          |Sensor_In_10_Pin);

    HAL_GPIO_DeInit(GPIOB, Sensor_In_11_Pin|Sensor_In_12_Pin);

    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(hadc->Instance==ADC2)
  {
    /* USER CODE BEGIN ADC2_MspDeInit 0 */

    /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PC0     ------> ADC2_INP10
    PC1     ------> ADC2_INP11
    PA2     ------> ADC2_INP14
    PA3     ------> ADC2_INP15
    PA4     ------> ADC2_INP18
    PA6     ------> ADC2_INP3
    PA7     ------> ADC2_INP7
    PC4     ------> ADC2_INP4
    PC5     ------> ADC2_INP8
    */
    HAL_GPIO_DeInit(GPIOC, Bat_Voltage_Pin|Sensor_In_1_Pin|Sensor_In_4_Pin|Sensor_In_5_Pin);

    HAL_GPIO_DeInit(GPIOA, Sensor_In_6_Pin|Sensor_In_7_Pin|Sensor_In_8_Pin|Sensor_In_9_Pin
                          |Sensor_In_10_Pin);

    /* USER CODE BEGIN ADC2_MspDeInit 1 */

    /* USER CODE END ADC2_MspDeInit 1 */
  }
  else if(hadc->Instance==ADC3)
  {
    /* USER CODE BEGIN ADC3_MspDeInit 0 */

    /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();

    /**ADC3 GPIO Configuration
    PC0     ------> ADC3_INP10
    PC1     ------> ADC3_INP11
    PC2_C     ------> ADC3_INP0
    PC3_C     ------> ADC3_INP1
    */
    HAL_GPIO_DeInit(GPIOC, Bat_Voltage_Pin|Sensor_In_1_Pin);

    /* USER CODE BEGIN ADC3_MspDeInit 1 */

    /* USER CODE END ADC3_MspDeInit 1 */
  }

}

/**
  * @brief FDCAN MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hfdcan: FDCAN handle pointer
  * @retval None
  */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hfdcan->Instance==FDCAN1)
  {
    /* USER CODE BEGIN FDCAN1_MspInit 0 */

    /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN FDCAN1_MspInit 1 */

    /* USER CODE END FDCAN1_MspInit 1 */

  }

}

/**
  * @brief FDCAN MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hfdcan: FDCAN handle pointer
  * @retval None
  */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan)
{
  if(hfdcan->Instance==FDCAN1)
  {
    /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

    /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

    /* USER CODE END FDCAN1_MspDeInit 1 */
  }

}

/**
  * @brief I2C MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
    /* USER CODE BEGIN I2C1_MspInit 0 */

    /* USER CODE END I2C1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
    /* USER CODE BEGIN I2C1_MspInit 1 */

    /* USER CODE END I2C1_MspInit 1 */
  }
  else if(hi2c->Instance==I2C2)
  {
    /* USER CODE BEGIN I2C2_MspInit 0 */

    /* USER CODE END I2C2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
    /* USER CODE BEGIN I2C2_MspInit 1 */

    /* USER CODE END I2C2_MspInit 1 */
  }

}

/**
  * @brief I2C MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1)
  {
    /* USER CODE BEGIN I2C1_MspDeInit 0 */

    /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    /* USER CODE BEGIN I2C1_MspDeInit 1 */

    /* USER CODE END I2C1_MspDeInit 1 */
  }
  else if(hi2c->Instance==I2C2)
  {
    /* USER CODE BEGIN I2C2_MspDeInit 0 */

    /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

    /* USER CODE BEGIN I2C2_MspDeInit 1 */

    /* USER CODE END I2C2_MspDeInit 1 */
  }

}

/**
  * @brief MMC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hmmc: MMC handle pointer
  * @retval None
  */
void HAL_MMC_MspInit(MMC_HandleTypeDef* hmmc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hmmc->Instance==SDMMC1)
  {
    /* USER CODE BEGIN SDMMC1_MspInit 0 */

    /* USER CODE END SDMMC1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
    PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SDMMC1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SDMMC1 GPIO Configuration
    PC8     ------> SDMMC1_D0
    PC9     ------> SDMMC1_D1
    PC10     ------> SDMMC1_D2
    PC11     ------> SDMMC1_D3
    PC12     ------> SDMMC1_CK
    PD2     ------> SDMMC1_CMD
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USER CODE BEGIN SDMMC1_MspInit 1 */

    /* USER CODE END SDMMC1_MspInit 1 */

  }

}

/**
  * @brief MMC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hmmc: MMC handle pointer
  * @retval None
  */
void HAL_MMC_MspDeInit(MMC_HandleTypeDef* hmmc)
{
  if(hmmc->Instance==SDMMC1)
  {
    /* USER CODE BEGIN SDMMC1_MspDeInit 0 */

    /* USER CODE END SDMMC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDMMC1_CLK_DISABLE();

    /**SDMMC1 GPIO Configuration
    PC8     ------> SDMMC1_D0
    PC9     ------> SDMMC1_D1
    PC10     ------> SDMMC1_D2
    PC11     ------> SDMMC1_D3
    PC12     ------> SDMMC1_CK
    PD2     ------> SDMMC1_CMD
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* USER CODE BEGIN SDMMC1_MspDeInit 1 */

    /* USER CODE END SDMMC1_MspDeInit 1 */
  }

}

/**
  * @brief SPI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hspi->Instance==SPI1)
  {
    /* USER CODE BEGIN SPI1_MspInit 0 */

    /* USER CODE END SPI1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PD7     ------> SPI1_MOSI
    PB4 (NJTRST)     ------> SPI1_MISO
    */
    GPIO_InitStruct.Pin = SPI_CJ125_SCK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(SPI_CJ125_SCK_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI_CJ125_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(SPI_CJ125_MOSI_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI_CJ125_MISO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(SPI_CJ125_MISO_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI1_MspInit 1 */

    /* USER CODE END SPI1_MspInit 1 */
  }
  else if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspInit 0 */

    /* USER CODE END SPI2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI2_MspInit 1 */

    /* USER CODE END SPI2_MspInit 1 */
  }

}

/**
  * @brief SPI MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI1)
  {
    /* USER CODE BEGIN SPI1_MspDeInit 0 */

    /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PD7     ------> SPI1_MOSI
    PB4 (NJTRST)     ------> SPI1_MISO
    */
    HAL_GPIO_DeInit(SPI_CJ125_SCK_GPIO_Port, SPI_CJ125_SCK_Pin);

    HAL_GPIO_DeInit(SPI_CJ125_MOSI_GPIO_Port, SPI_CJ125_MOSI_Pin);

    HAL_GPIO_DeInit(SPI_CJ125_MISO_GPIO_Port, SPI_CJ125_MISO_Pin);

    /* USER CODE BEGIN SPI1_MspDeInit 1 */

    /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspDeInit 0 */

    /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

    /* USER CODE BEGIN SPI2_MspDeInit 1 */

    /* USER CODE END SPI2_MspDeInit 1 */
  }

}

/**
  * @brief TIM_Base MSP Initialization
  * This function configures the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_base->Instance==TIM1)
  {
    /* USER CODE BEGIN TIM1_MspInit 0 */

    /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    PE13     ------> TIM1_CH3
    PE14     ------> TIM1_CH4
    */
    GPIO_InitStruct.Pin = Injector_1_Pin|Injector_2_Pin|Injector_3_Pin|Injector_4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM1_MspInit 1 */

    /* USER CODE END TIM1_MspInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
    /* USER CODE BEGIN TIM2_MspInit 0 */

    /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    */
    GPIO_InitStruct.Pin = CKP_Pin|CMP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM2_MspInit 1 */

    /* USER CODE END TIM2_MspInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspInit 0 */

    /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* USER CODE BEGIN TIM3_MspInit 1 */

    /* USER CODE END TIM3_MspInit 1 */
  }
  else if(htim_base->Instance==TIM4)
  {
    /* USER CODE BEGIN TIM4_MspInit 0 */

    /* USER CODE END TIM4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PD12     ------> TIM4_CH1
    PD13     ------> TIM4_CH2
    PD14     ------> TIM4_CH3
    PD15     ------> TIM4_CH4
    */
    GPIO_InitStruct.Pin = Input_Ig_1_Pin|Input_Ig_2_Pin|Input_Ig_3_Pin|Input_Ig_4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM4_MspInit 1 */

    /* USER CODE END TIM4_MspInit 1 */
  }
  else if(htim_base->Instance==TIM8)
  {
    /* USER CODE BEGIN TIM8_MspInit 0 */

    /* USER CODE END TIM8_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();
    /* USER CODE BEGIN TIM8_MspInit 1 */

    /* USER CODE END TIM8_MspInit 1 */
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspPostInit 0 */

    /* USER CODE END TIM3_MspPostInit 0 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PB5     ------> TIM3_CH2
    */
    GPIO_InitStruct.Pin = PWM_MCU_OUT_3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(PWM_MCU_OUT_3_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM3_MspPostInit 1 */

    /* USER CODE END TIM3_MspPostInit 1 */
  }
  else if(htim->Instance==TIM8)
  {
    /* USER CODE BEGIN TIM8_MspPostInit 0 */

    /* USER CODE END TIM8_MspPostInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM8 GPIO Configuration
    PC6     ------> TIM8_CH1
    PC7     ------> TIM8_CH2
    */
    GPIO_InitStruct.Pin = PWM_MCU_OUT_1_Pin|PWM_MCU_OUT_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM8_MspPostInit 1 */

    /* USER CODE END TIM8_MspPostInit 1 */
  }

}
/**
  * @brief TIM_Base MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
    /* USER CODE BEGIN TIM1_MspDeInit 0 */

    /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    PE13     ------> TIM1_CH3
    PE14     ------> TIM1_CH4
    */
    HAL_GPIO_DeInit(GPIOE, Injector_1_Pin|Injector_2_Pin|Injector_3_Pin|Injector_4_Pin);

    /* USER CODE BEGIN TIM1_MspDeInit 1 */

    /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
    /* USER CODE BEGIN TIM2_MspDeInit 0 */

    /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    */
    HAL_GPIO_DeInit(GPIOA, CKP_Pin|CMP_Pin);

    /* USER CODE BEGIN TIM2_MspDeInit 1 */

    /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspDeInit 0 */

    /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
    /* USER CODE BEGIN TIM3_MspDeInit 1 */

    /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM4)
  {
    /* USER CODE BEGIN TIM4_MspDeInit 0 */

    /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /**TIM4 GPIO Configuration
    PD12     ------> TIM4_CH1
    PD13     ------> TIM4_CH2
    PD14     ------> TIM4_CH3
    PD15     ------> TIM4_CH4
    */
    HAL_GPIO_DeInit(GPIOD, Input_Ig_1_Pin|Input_Ig_2_Pin|Input_Ig_3_Pin|Input_Ig_4_Pin);

    /* USER CODE BEGIN TIM4_MspDeInit 1 */

    /* USER CODE END TIM4_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM8)
  {
    /* USER CODE BEGIN TIM8_MspDeInit 0 */

    /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();
    /* USER CODE BEGIN TIM8_MspDeInit 1 */

    /* USER CODE END TIM8_MspDeInit 1 */
  }

}

/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(huart->Instance==UART7)
  {
    /* USER CODE BEGIN UART7_MspInit 0 */

    /* USER CODE END UART7_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_UART7_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART7 GPIO Configuration
    PE7     ------> UART7_RX
    PE8     ------> UART7_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* USER CODE BEGIN UART7_MspInit 1 */

    /* USER CODE END UART7_MspInit 1 */
  }
  else if(huart->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspInit 0 */

    /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN USART1_MspInit 1 */

    /* USER CODE END USART1_MspInit 1 */
  }
  else if(huart->Instance==USART3)
  {
    /* USER CODE BEGIN USART3_MspInit 0 */

    /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USER CODE BEGIN USART3_MspInit 1 */

    /* USER CODE END USART3_MspInit 1 */
  }

}

/**
  * @brief UART MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==UART7)
  {
    /* USER CODE BEGIN UART7_MspDeInit 0 */

    /* USER CODE END UART7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART7_CLK_DISABLE();

    /**UART7 GPIO Configuration
    PE7     ------> UART7_RX
    PE8     ------> UART7_TX
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7|GPIO_PIN_8);

    /* USER CODE BEGIN UART7_MspDeInit 1 */

    /* USER CODE END UART7_MspDeInit 1 */
  }
  else if(huart->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspDeInit 0 */

    /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USER CODE BEGIN USART1_MspDeInit 1 */

    /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(huart->Instance==USART3)
  {
    /* USER CODE BEGIN USART3_MspDeInit 0 */

    /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);

    /* USER CODE BEGIN USART3_MspDeInit 1 */

    /* USER CODE END USART3_MspDeInit 1 */
  }

}

/**
  * @brief WWDG MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hwwdg: WWDG handle pointer
  * @retval None
  */
void HAL_WWDG_MspInit(WWDG_HandleTypeDef* hwwdg)
{
  if(hwwdg->Instance==WWDG1)
  {
    /* USER CODE BEGIN WWDG1_MspInit 0 */

    /* USER CODE END WWDG1_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCCEx_WWDGxSysResetConfig(RCC_WWDG1);
    __HAL_RCC_WWDG1_CLK_ENABLE();
    /* USER CODE BEGIN WWDG1_MspInit 1 */

    /* USER CODE END WWDG1_MspInit 1 */

  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
