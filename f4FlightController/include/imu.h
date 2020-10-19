#ifndef _IMU_H
#define _IMU_H

#include "Arduino.h"

extern uint8_t IMU_ADDRESS;

extern float time_since_last_main_loop;

extern int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
extern float gyro_x_val, gyro_y_val, gyro_z_val;
extern float roll_angle, pitch_angle, yaw_angle;
extern float gyro_x_calibration, gyro_y_calibration, gyro_z_calibration;
extern float total_gyro_x_angle, total_gyro_y_angle, total_gyro_z_angle;
extern float acc_x_calibration, acc_y_calibration, acc_z_calibration;
extern int16_t imu_temp;
extern int16_t raw_acc_x, raw_acc_y, raw_acc_z;
extern float acc_x_val, acc_y_val, acc_z_val;
extern float acc_magnitude;

void SetupIMU(void);
void CalibrateIMU(void);
void ReadIMU(bool calibrating);
void ResetTimers(void);

#endif