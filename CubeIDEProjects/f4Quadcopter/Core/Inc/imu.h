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
void Calibrate_IMU(void);
void Read_IMU(uint8_t is_calibrating);

extern int16_t raw_gyro_acc_data[6];
extern float gyro_x, gyro_y, gyro_z;
extern float gyro_x_angle, gyro_y_angle, gyro_z_angle;
extern float acc_magnitude, acc_x, acc_y, acc_z, acc_magnitude_at_start;
extern float acc_x_g, acc_y_g, acc_z_g, acc_z_start;

#endif /* INC_IMU_H_ */
