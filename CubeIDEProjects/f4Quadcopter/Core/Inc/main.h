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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

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

extern uint8_t read_flag;
extern uint32_t how_long_to_loop_main;
extern uint32_t main_cycle_counter;
extern float how_long_to_loop_modifier;
extern volatile int32_t ppm_channels[6];
extern uint32_t main_loop_timer;

extern uint8_t altitude_hold_flag;
extern uint8_t gps_hold_flag;
extern uint8_t last_gps_hold_flag;
extern uint8_t gps_waypoint_flag;
extern uint8_t optical_flow_flag;
extern uint8_t new_camera_data;

extern uint32_t camera_request_timer;
extern uint8_t camera_receive_flag;
extern uint8_t camera_waiting_flag;
extern uint8_t camera_receive_buf[32];
extern uint8_t camera_send_buf[32];
extern float camera_displacement_x;
extern float camera_displacement_y;
extern float camera_velocity_x;
extern float camera_velocity_y;
extern int32_t camera_framerate;

extern int32_t raw_camera_x_velocity;
extern int32_t raw_camera_y_velocity;

extern int32_t last_camera_x_velocity;
extern int32_t last_camera_y_velocity;

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void ReadCamera();
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
uint32_t GetMicros();
uint32_t GetMillis();
uint32_t GetMillisDifference(uint32_t *timer_counter_to_use);
uint32_t GetMicrosDifference(uint32_t *timer_counter_to_use);
//void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
