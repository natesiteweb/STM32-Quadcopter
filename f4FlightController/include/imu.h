#ifndef _IMU_H
#define _IMU_H

#include "Arduino.h"

extern uint8_t IMU_ADDRESS;

extern int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
extern float gyro_x_val, gyro_y_val, gyro_z_val;
extern int16_t imu_temp;
extern int16_t raw_acc_x, raw_acc_y, raw_acc_z;

void SetupIMU();
void ReadIMU();

#endif