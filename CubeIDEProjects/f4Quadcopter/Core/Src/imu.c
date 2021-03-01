/*
 * imu.c
 *
 *  Created on: Feb 28, 2021
 *      Author: Nate
 */

#include "telemetry.h"
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "math.h"
#include "stdlib.h"

static const uint8_t GYRO_ADDR = 0x68 << 1;

/*
 * Gryo and Accelerometer Vars
 */
uint8_t imu_setup_buffer[30];
HAL_StatusTypeDef ret;
uint8_t raw_gyro_acc_buffer[14];
int16_t raw_gyro_acc_data[6];//0:2 gyro xyz, 3:5 accel xyz
int16_t raw_imu_temp;
float gyro_x, gyro_y, gyro_z;
float acc_magnitude, acc_x, acc_y, acc_z;

void Setup_IMU()
{
	imu_setup_buffer[0] = 0x6B;
	imu_setup_buffer[1] = 0x00;
	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, imu_setup_buffer, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		//strcpy((char*)buf, "Error Tx\r\n");
	}

	HAL_Delay(10);

	imu_setup_buffer[0] = 0x1B;
	imu_setup_buffer[1] = 0x08;
	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, imu_setup_buffer, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		//strcpy((char*)buf, "Error Tx\r\n");
	}

	HAL_Delay(10);

	imu_setup_buffer[0] = 0x1A;
	imu_setup_buffer[1] = 0x03;
	ret = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, imu_setup_buffer, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		//strcpy((char*)buf, "Error Tx\r\n");
	}
}

void Read_IMU()
{
	HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, (uint8_t *)raw_gyro_acc_buffer, 14, HAL_MAX_DELAY);//Implement failure logic later
	raw_gyro_acc_data[3] = (int16_t)((raw_gyro_acc_buffer[0] << 8) | (raw_gyro_acc_buffer[1]));	//Acc X
	raw_gyro_acc_data[4] = (int16_t)((raw_gyro_acc_buffer[2] << 8) | (raw_gyro_acc_buffer[3]));	//Acc Y
	raw_gyro_acc_data[5] = (int16_t)((raw_gyro_acc_buffer[4] << 8) | (raw_gyro_acc_buffer[5]));	//Acc Z
	raw_imu_temp = (int16_t)((raw_gyro_acc_buffer[6] << 8) | (raw_gyro_acc_buffer[7]));
	raw_gyro_acc_data[0] = (int16_t)((raw_gyro_acc_buffer[8] << 8) | (raw_gyro_acc_buffer[9]));	//Gyro X
	raw_gyro_acc_data[1] = (int16_t)((raw_gyro_acc_buffer[10] << 8) | (raw_gyro_acc_buffer[11]));	//Gyro Y
	raw_gyro_acc_data[2] = (int16_t)((raw_gyro_acc_buffer[12] << 8) | (raw_gyro_acc_buffer[13]));	//Gyro Z

	gyro_x = (float)raw_gyro_acc_data[0] / 65.5;
	gyro_y = (float)raw_gyro_acc_data[1] / 65.5;
	gyro_z = (float)raw_gyro_acc_data[2] / 65.5;

	acc_magnitude = sqrt(((float)raw_gyro_acc_data[3] * (float)raw_gyro_acc_data[3]) + ((float)raw_gyro_acc_data[4] * (float)raw_gyro_acc_data[4]) + ((float)raw_gyro_acc_data[5] * (float)raw_gyro_acc_data[5]));

	if(acc_magnitude != 0)
	{
		if(abs(raw_gyro_acc_data[4]) <= acc_magnitude)
		{
			acc_x = asin((float)raw_gyro_acc_data[4] / acc_magnitude) * 57.296;
		}
		else if(abs(raw_gyro_acc_data[3]) < acc_magnitude)
		{
			acc_y = asin((float)raw_gyro_acc_data[3] / acc_magnitude) * 57.296;
		}
	}
}
