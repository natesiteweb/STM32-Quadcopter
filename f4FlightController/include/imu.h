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

extern float z_acc_fast[30], z_acc_slow[50], z_acc_fast_total, z_acc_slow_total;
extern int z_acc_fast_reading_index, z_acc_slow_reading_index;
extern float acc_magnitude_at_start;

extern uint32_t takeoff_timer;

void SetupIMU(void);
void CalibrateIMU(void);
void ReadIMU(bool calibrating);
void ResetTimers(void);

#endif