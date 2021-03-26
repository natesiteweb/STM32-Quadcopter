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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAM_D0_Pin GPIO_PIN_0
#define CAM_D0_GPIO_Port GPIOA
#define CAM_D1_Pin GPIO_PIN_1
#define CAM_D1_GPIO_Port GPIOA
#define CAM_D2_Pin GPIO_PIN_2
#define CAM_D2_GPIO_Port GPIOA
#define CAM_D3_Pin GPIO_PIN_3
#define CAM_D3_GPIO_Port GPIOA
#define CAM_D4_Pin GPIO_PIN_4
#define CAM_D4_GPIO_Port GPIOA
#define CAM_D5_Pin GPIO_PIN_5
#define CAM_D5_GPIO_Port GPIOA
#define CAM_D6_Pin GPIO_PIN_6
#define CAM_D6_GPIO_Port GPIOA
#define CAM_D7_Pin GPIO_PIN_7
#define CAM_D7_GPIO_Port GPIOA
#define HSYNC_Pin GPIO_PIN_0
#define HSYNC_GPIO_Port GPIOB
#define VSYNC_Pin GPIO_PIN_1
#define VSYNC_GPIO_Port GPIOB
#define PCLK_Pin GPIO_PIN_3
#define PCLK_GPIO_Port GPIOB
#define SPI_RST_Pin GPIO_PIN_4
#define SPI_RST_GPIO_Port GPIOB
#define SPI_CS_Pin GPIO_PIN_5
#define SPI_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
