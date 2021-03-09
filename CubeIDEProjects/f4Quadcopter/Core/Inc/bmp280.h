/*
 * bmp280.h
 *
 *  Created on: Mar 3, 2021
 *      Author: Nate
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "stm32f4xx_hal.h"

void Setup_BMP280(void);
void Read_BMP280_Calibration_Data(void);
void Read_BMP280_PressureTemperature(void);
void Calibrate_BMP280(void);

float read_bmp_altitude;

#endif /* INC_BMP280_H_ */
