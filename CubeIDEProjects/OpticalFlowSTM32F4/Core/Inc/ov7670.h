/*
 * ov7670.h
 *
 *  Created on: Mar 26, 2021
 *      Author: Nate
 */

#ifndef INC_OV7670_H_
#define INC_OV7670_H_

#include "stm32f4xx_hal.h"

void Camera_Init(void);
uint8_t GetPixelIntensity(int16_t x, int16_t y);
uint8_t GetLastPixelIntensity(int16_t x, int16_t y);
uint32_t getMinimum(uint32_t *a, uint32_t n);
uint8_t reverse(uint8_t b);
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

extern uint8_t temp_data_line[144*144*2];
extern uint8_t temp_data_line2[144*144*2];
extern volatile uint32_t line_count;
extern volatile uint8_t frame_captured;
extern int32_t x_histogram[200];
extern int32_t y_histogram[200];
extern int32_t x_histogram_prev[200];
extern int32_t y_histogram_prev[200];

extern volatile uint32_t which_frame;

extern volatile uint8_t hsync_flag;

extern uint8_t data_out1, data_out2;

#endif /* INC_OV7670_H_ */
