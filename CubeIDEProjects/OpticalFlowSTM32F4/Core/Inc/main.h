/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;

extern uint8_t test_print_buffer[100];

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

uint32_t GetMicros();
uint32_t GetMillis();
uint32_t GetMillisDifference(uint32_t *timer_counter_to_use);
uint32_t GetMicrosDifference(uint32_t *timer_counter_to_use);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAM_RST_Pin GPIO_PIN_3
#define CAM_RST_GPIO_Port GPIOC
#define CAM_VSYNC_Pin GPIO_PIN_0
#define CAM_VSYNC_GPIO_Port GPIOB
#define CAM_HREF_Pin GPIO_PIN_8
#define CAM_HREF_GPIO_Port GPIOA
#define CAM_PCLK_Pin GPIO_PIN_10
#define CAM_PCLK_GPIO_Port GPIOA
#define TFT_BS_Pin GPIO_PIN_5
#define TFT_BS_GPIO_Port GPIOB
#define TFT_CS_Pin GPIO_PIN_8
#define TFT_CS_GPIO_Port GPIOB
#define TFT_RST_Pin GPIO_PIN_9
#define TFT_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
