/*
 * bmp280.c
 *
 *  Created on: Mar 3, 2021
 *      Author: Nate
 */

#include "main.h"
#include "i2c.h"
#include "bmp280.h"

static const uint8_t BMP280_ADDR = 0x76 << 1;

float seaLevelhPa = 1013.25;

uint8_t pressure_temperature_buffer[6];
int32_t adc_P;
int32_t adc_T;

int64_t pressureVar1, pressureVar2, pressureVarP;
float final_pressure;
float altitude_calibration;
float temp_altitude, read_bmp_altitude;

int32_t temperatureVar1, temperatureVar2;
float final_temperature;

typedef struct
{
    uint16_t dig_T1; /**< dig_T1 cal register. */
    int16_t dig_T2;  /**<  dig_T2 cal register. */
    int16_t dig_T3;  /**< dig_T3 cal register. */

    uint16_t dig_P1; /**< dig_P1 cal register. */
    int16_t dig_P2;  /**< dig_P2 cal register. */
    int16_t dig_P3;  /**< dig_P3 cal register. */
    int16_t dig_P4;  /**< dig_P4 cal register. */
    int16_t dig_P5;  /**< dig_P5 cal register. */
    int16_t dig_P6;  /**< dig_P6 cal register. */
    int16_t dig_P7;  /**< dig_P7 cal register. */
    int16_t dig_P8;  /**< dig_P8 cal register. */
    int16_t dig_P9;  /**< dig_P9 cal register. */
} bmp280_calib_data;

bmp280_calib_data _bmp280_calib;
int32_t t_fine;

void Setup_BMP280()
{
	uint8_t setup_data[4];
	setup_data[0] = 0xF5;
	setup_data[1] = 0x10;
	setup_data[2] = 0xF4;
	setup_data[3] = 0x57;

	HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDR, setup_data, 4, HAL_MAX_DELAY);
	Read_BMP280_Calibration_Data();
}

void Read_BMP280_Calibration_Data()
{
	HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0x88, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&_bmp280_calib, sizeof(bmp280_calib_data), HAL_MAX_DELAY);
}

void Read_BMP280_PressureTemperature()
{
	HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0xF7, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&pressure_temperature_buffer, 6, HAL_MAX_DELAY);

	adc_P = (pressure_temperature_buffer[0] << 16) | (pressure_temperature_buffer[1] << 8) | (pressure_temperature_buffer[2]);
	adc_P >>= 4;

	adc_T = (pressure_temperature_buffer[3] << 16) | (pressure_temperature_buffer[4] << 8) | (pressure_temperature_buffer[5]);
	adc_T >>= 4;

	temperatureVar1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) * ((int32_t)_bmp280_calib.dig_T2)) >> 11;
	temperatureVar2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) * ((int32_t)_bmp280_calib.dig_T3)) >> 14;

	t_fine = temperatureVar1 + temperatureVar2;

	final_temperature = (float)((t_fine * 5 + 128) >> 8) / 100;

	pressureVar1 = ((int64_t)t_fine) - 128000;
	pressureVar2 = pressureVar1 * pressureVar1 * (int64_t)_bmp280_calib.dig_P6;
	pressureVar2 = pressureVar2 + ((pressureVar1 * (int64_t)_bmp280_calib.dig_P5) << 17);
	pressureVar2 = pressureVar2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
	pressureVar1 = ((pressureVar1 * pressureVar1 * (int64_t)_bmp280_calib.dig_P3) >> 8) + ((pressureVar1 * (int64_t)_bmp280_calib.dig_P2) << 12);
	pressureVar1 = (((((int64_t)1) << 47) + pressureVar1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

	if (pressureVar1 != 0)
	{
		pressureVarP = 1048576 - adc_P;
		pressureVarP = (((pressureVarP << 31) - pressureVar2) * 3125) / pressureVar1;
		pressureVar1 = (((int64_t)_bmp280_calib.dig_P9) * (pressureVarP >> 13) * (pressureVarP >> 13)) >> 25;
		pressureVar2 = (((int64_t)_bmp280_calib.dig_P8) * pressureVarP) >> 19;

		pressureVarP = ((pressureVarP + pressureVar1 + pressureVar2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);

		final_pressure = (float)pressureVarP / 256;
		final_pressure /= 100;

		temp_altitude = (44330 * (1.0 - pow(final_pressure / seaLevelhPa, 0.1903)));
		read_bmp_altitude = temp_altitude - altitude_calibration;
	}
}

void Calibrate_BMP280()
{
	altitude_calibration = 0;

	for(int i = 0; i < 30; i++)
	{
		Read_BMP280_PressureTemperature();
		altitude_calibration += temp_altitude;
		HAL_Delay(50);
	}

	altitude_calibration /= 30.00;
}
