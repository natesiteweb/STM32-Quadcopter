/*
 * compass.c
 *
 *  Created on: Mar 23, 2021
 *      Author: Nate
 */

#include "compass.h"
#include "telemetry.h"
#include "imu.h"
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "math.h"
#include "stdlib.h"
#include "eeprom.h"

static const uint8_t COMPASS_ADDR = 0x1E << 1;

uint8_t raw_compass_data[6];

int16_t compass_x_min = 0, compass_x_max = 0;
int16_t compass_y_min = 0, compass_y_max = 0;
int16_t compass_z_min = 0, compass_z_max = 0;

uint32_t calibrate_compass_timer;

int16_t compassX, compassY, compassZ;
float compass_heading = 0;
float comX = 0, comY = 0, comZ = 0;
float compass_offset_x = 0, compass_offset_y = 0, compass_offset_z = 0;
float y_scale, z_scale;

float heading_difference_return = 0;

void Setup_Compass()
{
	uint8_t setup_data[4];
	setup_data[0] = 0x00;
	setup_data[1] = 0x78;
	setup_data[2] = 0x20;
	setup_data[3] = 0x00;

	HAL_I2C_Master_Transmit(&hi2c1, COMPASS_ADDR, setup_data, 4, HAL_MAX_DELAY);
	Calculate_Compass_Calibration();

	HAL_Delay(5);

	Read_Compass();

	gyro_z_angle = compass_heading;
}

float x_angle_offset = 1.00;//2.00
float y_angle_offset = 2.00;//3.00

void Read_Compass()
{
	HAL_I2C_Mem_Read(&hi2c1, COMPASS_ADDR, 0x03, I2C_MEMADD_SIZE_8BIT,  (uint8_t *)&raw_compass_data, 6, HAL_MAX_DELAY);

	compassY = (int16_t)((raw_compass_data[0] << 8) | raw_compass_data[1]);
	compassZ = (int16_t)((raw_compass_data[2] << 8) | raw_compass_data[3]);
	compassX = (int16_t)((raw_compass_data[4] << 8) | raw_compass_data[5]);

	compassY += compass_offset_y;
	compassY *= y_scale;

	compassZ += compass_offset_z;
	compassZ *= z_scale;

	compassX += compass_offset_x;

	comX = ((float)compassX * cos((gyro_y_angle + y_angle_offset) * 0.0174533)) + ((float)compassY * sin((gyro_x_angle + x_angle_offset) * 0.0174533) * sin((gyro_y_angle + y_angle_offset) * 0.0174533)) - ((float)compassZ * cos((gyro_x_angle + x_angle_offset) * 0.0174533) * sin((gyro_y_angle + y_angle_offset) * 0.0174533));

	comY = ((float)compassY * cos((gyro_x_angle + x_angle_offset) * 0.0174533)) + ((float)compassZ * sin((gyro_x_angle + x_angle_offset) * 0.0174533));

	if (comY < 0)
		compass_heading = 180 + (180 + ((atan2f((float)comY, (float)comX)) * 57.29577));
	else
		compass_heading = atan2f((float)comY, (float)comX) * 57.29577;

	compass_heading = -compass_heading;
	//compass_heading -= 5;

	if (compass_heading < 0)
		compass_heading += 360;
	else if (compass_heading >= 360)
		compass_heading -= 360;
}

void Calibrate_Compass()
{
	compass_x_min = 0;
	compass_x_max = 0;
	compass_y_min = 0;
	compass_y_max = 0;
	compass_z_min = 0;
	compass_z_max = 0;

	calibrate_compass_timer = GetMillis();

	while(GetMillisDifference(&calibrate_compass_timer) < 10000)
	{
		HAL_I2C_Mem_Read(&hi2c1, COMPASS_ADDR, 0x03, I2C_MEMADD_SIZE_8BIT,  (uint8_t *)&raw_compass_data, 6, HAL_MAX_DELAY);

		compassY = (int16_t)((raw_compass_data[0] << 8) | raw_compass_data[1]);
		compassZ = (int16_t)((raw_compass_data[2] << 8) | raw_compass_data[3]);
		compassX = (int16_t)((raw_compass_data[4] << 8) | raw_compass_data[5]);

		if (compassX > compass_x_max)
			compass_x_max = compassX;
		else if (compassX < compass_x_min)
			compass_x_min = compassX;

		if (compassZ > compass_z_max)
			compass_z_max = compassZ;
		else if (compassZ < compass_z_min)
			compass_z_min = compassZ;

		if (compassY > compass_y_max)
			compass_y_max = compassY;
		else if (compassY < compass_y_min)
			compass_y_min = compassY;

		HAL_Delay(5);
	}

	EEPROM_Clear_Buffer();
	eeprom_write_buffer_width = 2;
	EEPROM_Write_Buffer((uint8_t *)&compass_x_min, 2);
	EEPROM_Write_Buffer((uint8_t *)&compass_x_max, 2);
	EEPROM_Write_Buffer((uint8_t *)&compass_y_min, 2);
	EEPROM_Write_Buffer((uint8_t *)&compass_y_max, 2);
	EEPROM_Write_Buffer((uint8_t *)&compass_z_min, 2);
	EEPROM_Write_Buffer((uint8_t *)&compass_z_max, 2);
	EEPROM_Save_Page(64);

	Calculate_Compass_Calibration();

	Read_Compass();

	gyro_z_angle = compass_heading;
}

void Calculate_Compass_Calibration()
{
    y_scale = (float)(compass_x_max - compass_x_min) / (float)(compass_y_max - compass_y_min);
    z_scale = (float)(compass_x_max - compass_x_min) / (float)(compass_z_max - compass_z_min);

    compass_offset_x = (float)(compass_x_max - compass_x_min) / 2 - compass_x_max;
    compass_offset_y = ((float)(compass_y_max - compass_y_min) / 2 - compass_y_max) * y_scale;
    compass_offset_z = ((float)(compass_z_max - compass_z_min) / 2 - compass_z_max) * z_scale;
}

void CalculateHeadingDifference(float ang1, float ang2) //ang1 is setpoint, ang2 is current
{
    heading_difference_return = ang1 - ang2;

    if (heading_difference_return < -180 || heading_difference_return > 180)
    {
        if (ang2 > 180)
            ang2 -= 180;
        else
            ang2 += 180;

        if (ang1 > 180)
            ang1 -= 180;
        else
            ang1 += 180;

        heading_difference_return = ang1 - ang2;
    }
}
