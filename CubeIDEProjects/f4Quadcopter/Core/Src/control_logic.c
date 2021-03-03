/*
 * control_logic.c
 *
 *  Created on: Mar 2, 2021
 *      Author: Nate
 */


#include "telemetry.h"
#include "main.h"
#include "gpio.h"
#include "string.h"
#include "imu.h"
#include "stdlib.h"

float kp_roll = 1.1, kp_pitch, kp_yaw = 1.3;
float ki_roll = 0.001, ki_pitch, ki_yaw = 0.001;
float kd_roll = 6, kd_pitch, kd_yaw = 0;

int32_t pid_roll_output, pid_pitch_output, pid_yaw_output;
float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
float pid_roll_i, pid_pitch_i, pid_yaw_i;
int32_t esc1_output, esc2_output, esc3_output, esc4_output;
float pid_error_temp;
float pid_roll_last_error, pid_pitch_last_error, pid_yaw_last_error;

void Motor_PID()
{
	pid_error_temp = pid_roll_setpoint - gyro_x;
	pid_roll_i += (pid_error_temp * ki_roll);

	if(pid_roll_i > 300)
		pid_roll_i = 300;
	else if(pid_roll_i < -300)
		pid_roll_i = -300;

	pid_roll_output = (pid_error_temp * kp_roll) + pid_roll_i + ((pid_error_temp - pid_roll_last_error) * kd_roll);

	pid_roll_last_error = pid_error_temp;

	if(pid_roll_output > 300)
		pid_roll_output = 300;
	else if(pid_roll_output < -300)
		pid_roll_output = -300;

	pid_error_temp = pid_pitch_setpoint - gyro_y;
	pid_pitch_i += (pid_error_temp * ki_pitch);

	if(pid_pitch_i > 300)
		pid_pitch_i = 300;
	else if(pid_pitch_i < -300)
		pid_pitch_i = -300;

	pid_pitch_output = (pid_error_temp * kp_pitch) + pid_pitch_i + ((pid_error_temp - pid_pitch_last_error) * kd_pitch);

	pid_pitch_last_error = pid_error_temp;

	if(pid_pitch_output > 300)
		pid_pitch_output = 300;
	else if(pid_pitch_output < -300)
		pid_pitch_output = -300;

	pid_error_temp = pid_yaw_setpoint - gyro_z;
	pid_yaw_i += (pid_error_temp * ki_yaw);

	if(pid_yaw_i > 300)
		pid_yaw_i = 300;
	else if(pid_yaw_i < -300)
		pid_yaw_i = -300;

	pid_yaw_output = (pid_error_temp * kp_yaw) + pid_yaw_i + ((pid_error_temp - pid_yaw_last_error) * kd_yaw);

	pid_yaw_last_error = pid_error_temp;

	if(pid_yaw_output > 300)
		pid_yaw_output = 300;
	else if(pid_yaw_output < -300)
		pid_yaw_output = -300;

}
