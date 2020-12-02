/*
 * mainlogic.c
 *
 *  Created on: Sep 17, 2020
 *      Author: Nate
 */
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

#include "usbd_cdc_if.h"
#include "string.h"

uint8_t data[6] = "hello\n";

HAL_StatusTypeDef ret;
static const uint8_t GYRO_ADDR = 0x68 << 1;
uint8_t buf[40];
//int16_t val;
volatile uint32_t val;
volatile uint32_t i2c_transmit_timer = 0;
float gyro_x;

uint8_t read_flag = 0;
uint8_t send_buffer[2] = { 0x01, 0x01 };

void main_setup()
{
	HAL_Delay(6000);

	buf[0] = 0x6B;
	buf[1] = 0x00;
	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		strcpy((char*)buf, "Error Tx\r\n");
	}

	HAL_Delay(10);

	buf[0] = 0x1B;
	buf[1] = 0x08;
	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		strcpy((char*)buf, "Error Tx\r\n");
	}

	HAL_Delay(10);

	buf[0] = 0x1A;
	buf[1] = 0x03;
	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, buf, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		strcpy((char*)buf, "Error Tx\r\n");
	}

	CDC_Transmit_FS(buf, strlen((char*)buf));
}

void main_loop()
{
	HAL_Delay(50);

	if(read_flag == 1)
	{
		read_flag = 0;

		i2c_transmit_timer = HAL_GetTick();
		//i2c_transmit_timer = DWT->CYCCNT * (HAL_RCC_GetHCLKFreq() / 1000000);
		//HAL_Delay(10);
		HAL_I2C_Master_Transmit_DMA(&hi2c2, (uint8_t)(0x04 << 1), (uint8_t *)send_buffer, 2);
		//HAL_I2C_Master_Transmit();
		//HAL_I2C_Master_Transmit_IT();
		//HAL_Delay(5);
		//val = ((DWT->CYCCNT * (HAL_RCC_GetHCLKFreq() / 1000000)) - i2c_transmit_timer);
		val = (HAL_GetTick() - i2c_transmit_timer);

		//val = (int16_t)((buf[0] << 8) | (buf[1]));
		//gyro_x = val;
		//itoa(val, buf, 10);
		//itoa(val, buf, 10);
		/*if(val > 0)
		{
			sprintf((char*)buf, "%s", "1\r\n");
		}
		else
		{
			sprintf((char*)buf, "%s", "0\r\n");
		}*/
		sprintf((char*)buf, "%lu%s", val, "\r\n");
		//snprintf((char*)buf, 0, "%lu%s", val, "\r\n");

		CDC_Transmit_FS(buf, strlen((char*)buf));
		HAL_Delay(10);
		//i2c_transmit_timer = DWT->CYCCNT * (HAL_RCC_GetHCLKFreq() / 1000000);
		//HAL_I2C_Master_Receive_IT(hi2c, DevAddress, pData, Size)
	}

	//ret = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDR, 0x43, 1, buf, 2, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read_IT(&hi2c1, GYRO_ADDR, 0x43, 1, buf, 2);
	/*if(ret != HAL_OK)
	{
		strcpy((char*)buf, "Error Rx\r\n");
	}
	else
	{
		val = (int16_t)((buf[0] << 8) | (buf[1]));
		gyro_x = val;
		itoa(val, buf, 10);
		sprintf((char*)buf, "%s%s", buf, "\r\n");
	}*/
}
