#ifndef _IMU_H
#define _IMU_H

#include "Arduino.h"

extern int16_t raw_gyro_x;
extern float gyro_x_val;

void SetupIMU(uint8_t IMU_ADDRESS);
void ReadIMU(uint8_t IMU_ADDRESS);

#endif