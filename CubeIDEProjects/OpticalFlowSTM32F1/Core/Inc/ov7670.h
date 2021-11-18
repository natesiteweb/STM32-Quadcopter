/*
 * ov7670.h
 *
 *  Created on: Mar 26, 2021
 *      Author: Nate
 */

#ifndef INC_OV7670_H_
#define INC_OV7670_H_

#include "stm32f1xx_hal.h"

void Camera_Init(void);
uint8_t reverse(uint8_t b);
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

extern uint8_t temp_data_line[16000];
extern volatile uint32_t line_count;
extern volatile uint32_t lines_captured;
extern volatile uint8_t frame_captured;

extern volatile uint8_t hsync_flag;

extern uint8_t data_out1, data_out2;

#endif /* INC_OV7670_H_ */
