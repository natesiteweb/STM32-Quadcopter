/*
 * control_logic.h
 *
 *  Created on: Mar 2, 2021
 *      Author: Nate
 */

#ifndef INC_CONTROL_LOGIC_H_
#define INC_CONTROL_LOGIC_H_

#include "stm32f4xx_hal.h"

void Calculate_Attitude(void);
void Motor_PID(void);
void Calculate_Motor_Outputs(void);
void Init_Altitude_Kalman(void);
void Calculate_Altitude_Filter(void);
void Calculate_Altitude_PID(void);
void GPS_PID(void);
void OpticalFlow_PID(void);
void Launch_Behavior(void);
void Land_Behavior(void);

typedef struct
{
	uint8_t row;
	uint8_t col;

	float data[4*4];	//max 4X4 matrix
} matrix;

matrix Multiple_Matrix(matrix *a, matrix *b);

extern int32_t pid_roll_output, pid_pitch_output, pid_yaw_output;
extern float kp_roll, kp_yaw;
extern float ki_roll, ki_yaw;
extern float kd_roll, kd_yaw;

extern int32_t esc1_output, esc2_output, esc3_output, esc4_output;

extern float calculated_bmp_altitude;
extern float pid_current_altitude_setpoint;
extern float pid_altitude_setpoint;

extern uint8_t launched, launching, landing;
extern uint8_t manual_mode;

extern float hover_throttle;
extern int32_t idle_throttle;
extern uint32_t launch_timer;

extern float slow_bmp_altitude;
extern float kalman_x;
extern float kalman_x_new, kalman_xa_new;
extern float kalman_gain;
extern float kalman_measured_vel;
extern float pid_altitude_setpoint;
extern float pid_alt_last_error;
extern int32_t altitude_pid_output;
extern float kp_alt, ki_alt, kd_alt;
extern float pid_alt_i;

extern double current_state[2*1];
extern double kalman_gain_matrix[2*1];

extern float current_state_float;
extern float kalman_gain_float;
extern double current_state_double;
extern double kalman_gain_double;


extern float pid_altitude_over_time_total, pid_altitude_over_time[20];
extern uint8_t pid_altitude_over_time_reading_index;

/*
 * Waypoints
 * 0: Home
 */
extern int32_t lat_mem[200], lon_mem[200];
extern float alt_mem[200];

extern uint8_t sat_count;
extern int32_t raw_gps_lat, raw_gps_lon;
extern uint8_t new_gps_data;
extern int32_t last_raw_gps_lat, last_raw_gps_lon;
extern int32_t current_lat_setpoint, current_lon_setpoint;
extern uint8_t gps_setpoint;

extern float calculated_lat_error, calculated_lon_error;
extern float lat_add, lon_add;

extern float lat_error_over_time_total, lon_error_over_time_total;
extern float lat_error_over_time[50], lon_error_over_time[50];
extern uint8_t gps_error_over_time_reading_index;

extern float last_calculated_lat_error, last_calculated_lon_error;

extern float kp_gps, kp_gps_actual, ki_gps, kd_gps;
extern float pid_gps_i;

extern int32_t gps_roll_modifier, gps_pitch_modifier;
extern float gps_roll_modifier_north, gps_pitch_modifier_north;

extern float camera_roll_modifier, camera_pitch_modifier;
extern float kp_camera, kp_camera_actual, ki_camera, kd_camera;
extern float pid_camera_x_i, pid_camera_y_i;
extern float last_camera_roll_error, last_camera_pitch_error;

enum
{
	LANDED = 0x00,
	LAUNCHED = 0x01
};

#endif /* INC_CONTROL_LOGIC_H_ */
