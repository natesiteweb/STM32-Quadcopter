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
uint8_t buf[20];
int16_t val;
float gyro_x;

void main_setup()
{
	HAL_Delay(10000);

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

	ret = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDR, 0x43, 1, buf, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		strcpy((char*)buf, "Error Rx\r\n");
	}
	else
	{
		val = (int16_t)((buf[0] << 8) | (buf[1]));
		gyro_x = val;
		itoa(val, buf, 10);
		sprintf((char*)buf, "%s%s", buf, "\r\n");
	}

	CDC_Transmit_FS(buf, strlen((char*)buf));
}
