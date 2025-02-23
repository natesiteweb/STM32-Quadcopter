/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
uint32_t GetMicros();
uint32_t GetMillis();
uint32_t GetMillisDifference(uint32_t *timer_counter_to_use);
uint32_t GetMicrosDifference(uint32_t *timer_counter_to_use);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SAT_TX_Pin GPIO_PIN_2
#define SAT_TX_GPIO_Port GPIOA
#define SAT_RX_Pin GPIO_PIN_3
#define SAT_RX_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOA
#define BLE_INT_Pin GPIO_PIN_1
#define BLE_INT_GPIO_Port GPIOB
#define BLE_TX_Pin GPIO_PIN_10
#define BLE_TX_GPIO_Port GPIOB
#define BLE_RX_Pin GPIO_PIN_11
#define BLE_RX_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_12
#define NRF_CE_GPIO_Port GPIOB
#define NRF_CSN_Pin GPIO_PIN_8
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_IRQ_Pin GPIO_PIN_11
#define NRF_IRQ_GPIO_Port GPIOA
#define NRF_IRQ_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
