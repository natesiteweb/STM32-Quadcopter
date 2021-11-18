/*
 * ov7670.c
 *
 *  Created on: Mar 26, 2021
 *      Author: Nate
 */

#include "ov7670.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "ST7735.h"
#include "GFX_FUNCTIONS.h"
#include "stm32f4xx_hal_i2c.h"
#include "i2c_sw.h"

static const uint8_t OV7670_ADDRESS = 0x42;

uint8_t frame1[128*128];
uint8_t frame2[128*128];
uint8_t frame3[128*128];
uint8_t frame4[128*128];
volatile uint32_t line_count = 0;
volatile uint8_t frame_captured = 0;

volatile uint8_t hsync_flag = 0;

uint8_t data_out1, data_out2;

void SCCBDelay(uint16_t t)
{
	__HAL_TIM_SET_COUNTER(&htim9, 0);
	while(__HAL_TIM_GET_COUNTER(&htim9) < (uint16_t)t);
}

static uint8_t ReadRegister(uint8_t reg)
{
	//HAL_I2C_Master_Transmit();
    while(I2C1->SR2 & I2C_SR2_BUSY);
    I2C1->CR1 |= I2C_CR1_START;
    while(~I2C1->SR1 & I2C_SR1_SB);
    I2C1->DR = OV7670_ADDRESS;
    while(~I2C1->SR1 & I2C_SR1_ADDR);
    I2C1->SR2;       // Dummy read
    I2C1->DR = reg;  // Write the register number to be read
    while(~I2C1->SR1 & (I2C_SR1_TXE | I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;

    // Read the register value
    while(I2C1->SR2 & I2C_SR2_BUSY);
    I2C1->CR1 |= I2C_CR1_START;
    while(~I2C1->SR1 & I2C_SR1_SB);
    I2C1->DR = OV7670_ADDRESS | 1;
    while(~I2C1->SR1 & I2C_SR1_ADDR);
    I2C1->SR2;       // Dummy read
    I2C1->CR1 |= I2C_CR1_STOP;
    while(~I2C1->SR1 & I2C_SR1_RXNE);
    uint8_t data = I2C1->DR;
    return data;
}

static void WriteRegister(uint8_t reg, uint8_t value)
{
    while(I2C1->SR2 & I2C_SR2_BUSY);
    I2C1->CR1 |= I2C_CR1_START;
    while(~I2C1->SR1 & I2C_SR1_SB);
    I2C1->DR = OV7670_ADDRESS;
    while(~I2C1->SR1 & I2C_SR1_ADDR);
    I2C1->SR2;          // Dummy read
    I2C1->DR = reg;     // Write the register number
    while(~I2C1->SR1 & (I2C_SR1_TXE | I2C_SR1_BTF));
    I2C1->DR = value;   // Write the register value
    while(~I2C1->SR1 & (I2C_SR1_TXE | I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;
}

void Camera_Init()
{
	HAL_Delay(50);

	ReadRegister(0x0A);


	// Disable timing resets
	WriteRegister(0x0F, 0x00);//REG_COM6

	//WriteRegister(0x3A, 0x04);

	// Set clock prescaler to 2
	//WriteRegister(0x11, 0x05);//REG_CLKRC//0x0B//0x17//0x05
	WriteRegister(0x11, 0x01);//REG_CLKRC//0x0B//0x17//0x05
	WriteRegister(0x6B, 0x80);//REG_DBLV//0xC0

	// Enable scaling
	WriteRegister(0x0C, 0x08);//REG_COM3//0x08//0x0C

	// Use QCIF output format
	WriteRegister(0x12, 0x08);//REG_COM7 | 0x04 for rgb

	//RGB565
	//WriteRegister(0x40, 0xC0 | 0x10);

	// Blank pixel clock during sync pulses
	//WriteRegister2(0x15, 0x20);//REG_COM10  | 0x40 change to hsync
	//WriteRegister(0x15, 0x20);


	// Enable pixel clock scaling
	/*WriteRegister(0x3E, 0x12);//REG_COM14//0x19
	WriteRegister(0x70, 0x3A);//SCALING_XSC
	WriteRegister(0x71, 0x35);//SCALING_YSC
	WriteRegister(0x72, 0x11);//SCALING_DCWCTR
	WriteRegister(0x73, 0xF1);//SCALING_PCLK_DIV
	WriteRegister(0xA2, 0x52);//SCALING_DELAY*/

	//if(ReadRegister(0x15) == 0x00)
	//	fillScreen(GREEN);

	//ST7735_SetRotation(0);
	//sprintf((char *)temp_data_line, "%hx", ReadRegister(0x15));
	//ST7735_WriteString(0, 0, (char *)temp_data_line, Font_11x18, RED,BLACK);

	//WriteRegister(0x73, 1);//REG_SCALING_PCLK_DIV
	/*DMA2_Stream1->PAR = (uint32_t)&(DCMI->DR);
	DMA2_Stream1->NDTR = (uint16_t)176*144;
	DMA2_Stream1->M0AR = (uint32_t)test_frame;
	DMA2_Stream1->CR |= DMA_SxCR_EN;
	DCMI->CR |= (uint32_t)(1 << 1);

	DCMI->IER = (uint32_t)(1);

	DCMI->CR |= (uint32_t)(1 << 14);
	DCMI->CR |= (uint32_t)(1);*/

}

uint8_t GetPixelIntensity(uint8_t *frame, int16_t x, int16_t y)
{
	uint8_t intensity = frame[x + (y * 128)];

	return intensity;
}

uint32_t getMinimum(uint32_t *a, uint32_t n)
{
  uint32_t i;
  uint32_t min_ind = 0;
  uint32_t min_err = a[min_ind];
  uint32_t min_err_tot = 0;
  for (i = 1; i < n; i++) {
    if (a[i] <= min_err) {
      min_ind = i;
      min_err = a[i];
      min_err_tot += min_err;
    }
  }
  //*min_error = min_err_tot;
  return min_ind;
}

uint8_t reverse(uint8_t b)
{
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

/*void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		lines_captured++;
		hsync_flag = 1;
		//HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
		//HAL_DMA_Abort_IT(htim2.hdma[TIM_DMA_ID_CC1]);
		//TIM2->DIER |= TIM_DIER_CC1DE;
		TIM2->DIER &= ~TIM_DIER_CC1DE;
		TIM2->DIER &= ~TIM_DIER_CC2IE;
		//TIM2->SR &= ~TIM_SR_CC2IF;

		DMA1_Channel5->CCR = 0;
		DMA1_Channel5->CNDTR = (uint16_t)128;
		DMA1_Channel5->CMAR = (uint32_t)(temp_data_line + (line_count * 128));
		DMA1_Channel5->CCR = DMA_CCR_PL | DMA_CCR_MINC | DMA_CCR_EN;
		//TIM2->DIER |= TIM_DIER_CC1DE;
		//TIM2->DIER |= TIM_DIER_CC2IE;
		//HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)temp_data_line, sizeof(temp_data_line));

		line_count++;

		if(line_count > 80)
		{
			lines_captured = 0;
			frame_captured = 1;

			TIM3->DIER &= ~TIM_DIER_CC3IE;
			TIM3->CR1 &= ~TIM_CR1_CEN;
			TIM2->CR1 &= ~TIM_CR1_CEN;
		}
		else
		{
			TIM2->DIER |= TIM_DIER_CC1DE;
			TIM2->DIER |= TIM_DIER_CC2IE;
		}

		//sprintf((char *)print_text_buffer, "%s", "tim2\n");
		//HAL_UART_Transmit(&huart1, print_text_buffer, strlen((char*)print_text_buffer), HAL_MAX_DELAY);
	}
	else if(htim == &htim3)
	{
		//sprintf((char *)print_text_buffer, "%lu%s", line_count, "\n");
		//HAL_UART_Transmit(&huart1, print_text_buffer, strlen((char*)print_text_buffer), HAL_MAX_DELAY);

		line_count = 0;
		//fillScreen(BLACK);

		if(~TIM2->CR1 & TIM_CR1_CEN)
		{
			//sprintf((char *)print_text_buffer, "%s", "tim3\n");
			//HAL_UART_Transmit(&huart1, print_text_buffer, strlen((char*)print_text_buffer), HAL_MAX_DELAY);

			//HAL_DMA_Start_IT(htim2.hdma[TIM_DMA_ID_CC1], (uint32_t)&(GPIOA->IDR), (uint32_t)temp_data_line, sizeof(temp_data_line));
			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
			HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)temp_data_line, 128);
			DMA1_Channel5->CPAR = (uint32_t)&(GPIOA->IDR);
			//TIM2->DIER = TIM_DIER_CC2IE;

			//TIM2->CR1 = TIM_CR1_CEN;

			//TIM3->CCR2;
			//TIM3->SR &= ~TIM_SR_CC3IF;
			return;
		}
		else
		{
			//HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
			//HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_3);
			//TIM2->DIER &= ~TIM_DIER_CC1DE;
		}

		//TIM3->CCR2;
		//TIM3->SR &= ~TIM_SR_CC3IF;
	}
}
*/
