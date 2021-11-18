/*
 * ov7670.h
 *
 *  Created on: Mar 26, 2021
 *      Author: Nate
 */

#ifndef INC_OV7670_H_
#define INC_OV7670_H_

#include "stm32f4xx_hal.h"

#define SCCB_SDA_SET GPIOB->BSRR |= (1UL << 7)
#define SCCB_SDA_CLR GPIOB->BSRR |= (1UL << 23)
#define SCCB_SCL_SET GPIOB->BSRR |= (1UL << 6)
#define SCCB_SCL_CLR GPIOB->BSRR |= (1UL << 22)

void SCCBDelay(uint16_t t);
void Camera_Init(void);
uint8_t GetPixelIntensity(uint8_t *frame, int16_t x, int16_t y);
uint32_t getMinimum(uint32_t *a, uint32_t n);
uint8_t reverse(uint8_t b);
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

extern I2C_HandleTypeDef hi2c1;

extern uint8_t frame1[128*128];
extern uint8_t frame2[128*128];
extern uint8_t frame3[128*128];
extern uint8_t frame4[128*128];
extern uint8_t *current_frame;
extern uint8_t *last_frame;
extern volatile uint32_t line_count;
extern volatile uint8_t frame_captured;

extern volatile uint8_t hsync_flag;

extern uint8_t data_out1, data_out2;

#endif /* INC_OV7670_H_ */
