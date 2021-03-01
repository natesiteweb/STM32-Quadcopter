/*
 * imu.h
 *
 *  Created on: Feb 28, 2021
 *      Author: Nate
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

void Setup_IMU(void);
void Read_IMU(void);

extern int16_t raw_gyro_acc_data[6];
extern float gyro_x, gyro_y, gyro_z;
extern float acc_magnitude, acc_x, acc_y, acc_z;

#endif /* INC_IMU_H_ */
