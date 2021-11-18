/*
 * control_logic.c
 *
 *  Created on: Mar 2, 2021
 *      Author: Nate
 */


#include "telemetry.h"
#include "control_logic.h"
#include "control_loop.h"
#include "main.h"
#include "gpio.h"
#include "string.h"
#include "imu.h"
#include "stdlib.h"
#include "math.h"
#include "bmp280.h"
#include "compass.h"
#include "LinearAlgebra/declareFunctions.h"

float kp_roll = 0, kp_yaw = 0;
float ki_roll = 0, ki_yaw = 0;
float kd_roll = 0, kd_yaw = 0;

int32_t pid_roll_output, pid_pitch_output, pid_yaw_output;
float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
float pid_roll_i, pid_pitch_i, pid_yaw_i;
int32_t esc1_output = 125, esc2_output = 125, esc3_output = 125, esc4_output = 125;
float pid_error_temp;
float pid_roll_last_error, pid_pitch_last_error, pid_yaw_last_error;
int32_t max_motor_pid_output = 45;

void Calculate_Attitude()
{
	gyro_x = (float)raw_gyro_acc_data[0] / 65.5;
	gyro_y = (float)raw_gyro_acc_data[1] / -65.5;
	gyro_z = (float)raw_gyro_acc_data[2] / -65.5;

	acc_magnitude = sqrt(((float)raw_gyro_acc_data[3] * (float)raw_gyro_acc_data[3]) + ((float)raw_gyro_acc_data[4] * (float)raw_gyro_acc_data[4]) + ((float)raw_gyro_acc_data[5] * (float)raw_gyro_acc_data[5]));

	if(acc_magnitude != 0)
	{
		if(abs(raw_gyro_acc_data[4]) < acc_magnitude)
		{
			acc_x = asin((float)raw_gyro_acc_data[4] / acc_magnitude) * 57.296;
		}

		if(abs(raw_gyro_acc_data[3]) < acc_magnitude)
		{
			acc_y = asin((float)raw_gyro_acc_data[3] / acc_magnitude) * 57.296;
		}
	}

	gyro_x_angle += (gyro_x) * ((float)how_long_to_loop_main / 1000000);
	gyro_y_angle += (gyro_y) * ((float)how_long_to_loop_main / 1000000);
	gyro_z_angle += (gyro_z) * ((float)how_long_to_loop_main / 1000000);

	gyro_x_angle += (gyro_y_angle * sin(gyro_z * 0.01745 * ((float)how_long_to_loop_main / 1000000)));
	gyro_y_angle -= (gyro_x_angle * sin(gyro_z * 0.01745 * ((float)how_long_to_loop_main / 1000000)));

	gyro_x_angle = (gyro_x_angle * 0.9975) + (acc_x * (1.0000 - 0.9975));
	gyro_y_angle = (gyro_y_angle * 0.9975) + (acc_y * (1.0000 - 0.9975));

	if(gyro_z_angle < 0)
		gyro_z_angle += 360;
	if(gyro_z_angle >= 360)
		gyro_z_angle -= 360;

	CalculateHeadingDifference(gyro_z_angle, compass_heading);

	if (heading_difference_return > 5 || heading_difference_return < -5)
		gyro_z_angle = compass_heading;
}

void Motor_PID()
{
	pid_roll_setpoint = 0;
	pid_pitch_setpoint = 0;
	pid_yaw_setpoint = 0;

	//Temp motor control
	if(ppm_channels[0] > 1505)
		pid_roll_setpoint = ppm_channels[0] - 1505;
	else if(ppm_channels[0] < 1495)
		pid_roll_setpoint = ppm_channels[0] - 1495;

	if(ppm_channels[1] > 1505)
		pid_pitch_setpoint = ppm_channels[1] - 1505;
	else if(ppm_channels[1] < 1495)
		pid_pitch_setpoint = ppm_channels[1] - 1495;

	if(ppm_channels[3] > 1505)
		pid_yaw_setpoint = ppm_channels[3] - 1505;
	else if(ppm_channels[3] < 1495)
		pid_yaw_setpoint = ppm_channels[3] - 1495;

	if(gps_hold_flag)
	{
		pid_roll_setpoint += gps_roll_modifier;
		pid_pitch_setpoint += gps_pitch_modifier;
	}

	if(optical_flow_flag)
	{
		pid_roll_setpoint += camera_roll_modifier;
		pid_pitch_setpoint += camera_pitch_modifier;
	}

	pid_roll_setpoint -= ((gyro_x_angle) * 15);
	pid_pitch_setpoint -= ((gyro_y_angle) * 15);

	pid_roll_setpoint /= 3.0;
	pid_pitch_setpoint /= 3.0;
	pid_yaw_setpoint /= 3.0;

	pid_error_temp = pid_roll_setpoint - gyro_x;
	pid_roll_i += (pid_error_temp * ki_roll * how_long_to_loop_modifier);

	if(pid_roll_i > max_motor_pid_output)
		pid_roll_i = max_motor_pid_output;
	else if(pid_roll_i < (max_motor_pid_output * -1))
		pid_roll_i = (max_motor_pid_output * -1);

	pid_roll_output = (pid_error_temp * kp_roll * how_long_to_loop_modifier) + pid_roll_i + ((pid_error_temp - pid_roll_last_error) * kd_roll * how_long_to_loop_modifier);

	pid_roll_last_error = pid_error_temp;

	if(pid_roll_output > max_motor_pid_output)
		pid_roll_output = max_motor_pid_output;
	else if(pid_roll_output < (max_motor_pid_output * -1))
		pid_roll_output = (max_motor_pid_output * -1);

	pid_error_temp = pid_pitch_setpoint - gyro_y;
	pid_pitch_i += (pid_error_temp * ki_roll * how_long_to_loop_modifier);

	if(pid_pitch_i > max_motor_pid_output)
		pid_pitch_i = max_motor_pid_output;
	else if(pid_pitch_i < (max_motor_pid_output * -1))
		pid_pitch_i = (max_motor_pid_output * -1);

	pid_pitch_output = (pid_error_temp * kp_roll * how_long_to_loop_modifier) + pid_pitch_i + ((pid_error_temp - pid_pitch_last_error) * kd_roll * how_long_to_loop_modifier);

	pid_pitch_last_error = pid_error_temp;

	if(pid_pitch_output > max_motor_pid_output)
		pid_pitch_output = max_motor_pid_output;
	else if(pid_pitch_output < (max_motor_pid_output * -1))
		pid_pitch_output = (max_motor_pid_output * -1);

	pid_error_temp = pid_yaw_setpoint - gyro_z;
	pid_yaw_i += (pid_error_temp * ki_yaw * how_long_to_loop_modifier);

	if(pid_yaw_i > max_motor_pid_output)
		pid_yaw_i = max_motor_pid_output;
	else if(pid_yaw_i < (max_motor_pid_output * -1))
		pid_yaw_i = (max_motor_pid_output * -1);

	pid_yaw_output = (pid_error_temp * kp_yaw * how_long_to_loop_modifier) + pid_yaw_i + ((pid_error_temp - pid_yaw_last_error) * kd_yaw * how_long_to_loop_modifier);

	pid_yaw_last_error = pid_error_temp;

	if(pid_yaw_output > max_motor_pid_output)
		pid_yaw_output = max_motor_pid_output;
	else if(pid_yaw_output < (max_motor_pid_output * -1))
		pid_yaw_output = (max_motor_pid_output * -1);
}

int32_t manual_throttle;
int32_t throttle_output;

void Calculate_Motor_Outputs()
{
	if(ppm_channels[2] < 1008)
		manual_throttle = 125;
	else
		manual_throttle = (ppm_channels[2] / 8);

	throttle_output = idle_throttle;

	if(altitude_hold_flag)
		throttle_output += altitude_pid_output;

	if(manual_mode)
	{
		throttle_output = manual_throttle;
	}

	if(ppm_channels[5] > 1300)
	{
		esc1_output = throttle_output + pid_roll_output + pid_pitch_output - pid_yaw_output;
		esc2_output = throttle_output - pid_roll_output + pid_pitch_output + pid_yaw_output;
		esc3_output = throttle_output - pid_roll_output - pid_pitch_output - pid_yaw_output;
		esc4_output = throttle_output + pid_roll_output - pid_pitch_output + pid_yaw_output;

		if(esc1_output > 250)
			esc1_output = 250;
		else if(esc1_output < 125)
			esc1_output = 125;

		if(esc2_output > 250)
			esc2_output = 250;
		else if(esc2_output < 125)
			esc2_output = 125;

		if(esc3_output > 250)
			esc3_output = 250;
		else if(esc3_output < 125)
			esc3_output = 125;

		if(esc4_output > 250)
			esc4_output = 250;
		else if(esc4_output < 125)
			esc4_output = 125;
	}
	else
	{
		pid_roll_setpoint = 0;
		pid_pitch_setpoint = 0;
		pid_yaw_setpoint = 0;

		pid_roll_output = 0;
		pid_pitch_output = 0;
		pid_yaw_output = 0;

		pid_roll_i = 0;
		pid_pitch_i = 0;
		pid_yaw_i = 0;

		esc1_output = 125;
		esc2_output = 125;
		esc3_output = 125;
		esc4_output = 125;
	}
}

uint32_t altitude_pid_timer;

float total_bmp_altitude;
float bmp_over_time[20];
uint8_t bmp_reading_index = 0;

float pressure_difference = 0;
float fast_bmp_altitude = 0;
float slow_bmp_altitude = 0;
float kalman_bmp_altitude = 0;

float pid_current_altitude_setpoint = 3;
float pid_altitude_setpoint = 3;

float pid_alt_last_error = 0;
int32_t altitude_pid_output = 0;

float kp_alt = 0, kp_alt_actual = 0, ki_alt = 0, kd_alt = 0;
float pid_alt_i = 0;

float pid_altitude_over_time_total = 0, pid_altitude_over_time[20];
uint8_t pid_altitude_over_time_reading_index = 0;

/*
 * Altitude Kalman Filter Testing
 */
uint32_t altitude_timer = 0;
float deltaT = 0;
float kalman_x = 0.5, kalman_last_x = 0;
float kalman_x_new = 0, kalman_xa_new = 0;
float kalman_xv = 0, kalman_xa = 0;
float kalman_measured_vel = 0, kalman_last_measured_x = 0;
float kalman_q = 0.0004, kalman_qa = 0.002;//0.00020;
float kalman_gain = 0.75, kalman_gain_a = 0.75;
float kalman_x_err = 0.2, kalman_xa_err = 0.2;
float kalman_measurement_err = 0.15, kalman_acc_measurement_err = 0.10;

/*
 * Test 2 with Matrices
 */

double current_state[2*1] = { 0, 0 }, last_state[2*1] = { 0, 0 };
double a_matrix[2*2] = { 1, 0, 0, 1 }, a_tran_matrix[2*2] = { 1, 0, 0, 1 };
double b_matrix[2*1];
double kalman_gain_matrix[2*1]  = { .99, .99};
double z_baro[1] = { 0 };
double predicted_process_covariance[2*2] = { 0.01, 0, 0, 0.01 };
double r_matrix[1*1] = { 0.1 }, q_matrix[2*2] = { 0.03, 0, 0, 0.001 };
double h_matrix[1*2] = { 1, 0 } , h_tran_matrix[2*1] = { 1, 0 };
double i_matrix[2*2] = { 1, 0, 0, 1 };
double g[2*2], f[2*2], m[2*2], n[2*2], o[2*2];		//Math placeholder matrix
float current_state_float = 0;
float kalman_gain_float = 0;
double current_state_double = 0;
double kalman_gain_double = 0;
/*matrix current_state;
matrix a_matrix;
matrix kalman_gain_matrix;
matrix predicted_process_covariance, last_predicted_process_covariance;
matrix r_matrix, q_matrix;*/

void Init_Altitude_Kalman()
{
	for(int i = 0; i < 2*2; i++)
	{
		g[i] = 0;
		predicted_process_covariance[i] = 0.1;
	}

	predicted_process_covariance[0] = 0.1;
	predicted_process_covariance[3] = 0.1;

	/*current_state.row = 3;
	current_state.col = 1;

	//kalman_gain_matrix.row

	for(int i = 0; i < 4*4; i++)
	{
		current_state.data[i] = 0;
	}*/
}

void Calculate_Altitude_Filter()
{
	//ACC upward is positive
	//Predict
	deltaT = (float)((float)GetMicrosDifference(&altitude_timer) * 0.00000100);
	//deltaT = 0.02;
	//SendDynamicVariable("Delta T", 3, (uint8_t *)&deltaT);
	if(deltaT > 0.2)
		deltaT = 0.2;

	altitude_timer = GetMicros();

	/*a_matrix[1] = (double)deltaT;
	a_tran_matrix[2] = (double)deltaT;

	b_matrix[0] = (double)(deltaT * deltaT * 0.5 * acc_z_g);
	b_matrix[1] = (double)(deltaT * acc_z_g);

	current_state[0] = last_state[0] + last_state[1] * (double)deltaT + (double)(acc_z_g * 0.5 * deltaT * deltaT);
	current_state[1] = last_state[1] + (double)(acc_z_g * deltaT);

	mul(a_matrix, predicted_process_covariance, false, g, 2, 2, 2);
	mul(g, a_tran_matrix, false, predicted_process_covariance, 2, 2, 2);

	for(int i = 0; i < 2*2; i++)
	{
		predicted_process_covariance[i] += q_matrix[i];
	}*/

	if((main_cycle_counter + 1) % 10 == 0)
	{
		/*z_baro[0] = (double)read_bmp_altitude;

		//State update
		mul(h_matrix, current_state, false, g, 1, 2, 1);
		f[0] = z_baro[0] - g[0];
		mul(kalman_gain_matrix, f, false, g, 2, 1, 1);
		current_state[0] += g[0];
		current_state[1] += g[1];

		//Covariance update
		mul(kalman_gain_matrix, h_matrix, false, g, 2, 1, 2);
		sub(i_matrix, g, f, 2, 2, 2);

		m[0] = f[0];
		m[1] = f[1];
		m[2] = f[2];
		m[3] = f[3];

		tran(m, 2, 2);

		mul(f, predicted_process_covariance, false, g, 2, 2, 2);
		mul(g, m, false, f, 2, 2, 2);

		n[0] = kalman_gain_matrix[0];
		n[1] = kalman_gain_matrix[1];
		n[2] = kalman_gain_matrix[2];
		n[3] = kalman_gain_matrix[3];

		tran(n, 2, 2);
		mul(kalman_gain_matrix, r_matrix, false, g, 2, 1, 1);
		mul(g, n, false, m, 2, 1, 2);

		predicted_process_covariance[0] = f[0] + m[0];

		//Kalman gain update

		mul(h_matrix, predicted_process_covariance, false, f, 1, 2, 2);
		mul(f, h_tran_matrix, false, m, 1, 2, 1);
		m[0] += r_matrix[0];
		m[0] = (double)1.0 / m[0];

		mul(predicted_process_covariance, h_tran_matrix, false, g, 2, 2, 1);
		mul(g, m, false, kalman_gain_matrix, 2, 1, 1);*/


		/*kalman_measured_vel = read_bmp_altitude - kalman_last_measured_x;
		kalman_measured_vel /= deltaT;

		if(kalman_x_err != kalman_measurement_err)
			kalman_gain = (kalman_x_err) / (kalman_x_err + kalman_measurement_err);

		if(kalman_xa_err != kalman_acc_measurement_err)
			kalman_gain_a = (kalman_xa_err) / (kalman_xa_err + kalman_acc_measurement_err);

		kalman_xa = kalman_xa_new;
		kalman_xv = ((kalman_xv + (kalman_xa * 9.81f * deltaT)));// + (kalman_measured_vel * deltaT * 0.05f);
		//kalman_x = kalman_last_x + (kalman_xv * deltaT) + (deltaT * deltaT * (acc_z_g * 9.81f * 0.5f));
		kalman_x = kalman_x_new + (kalman_xv * deltaT) + (deltaT * deltaT * kalman_xa * 0.5f);
		kalman_x_new = kalman_x + (kalman_gain * (read_bmp_altitude - kalman_x));
		kalman_xa_new = kalman_xa + (kalman_gain_a * (acc_z_g - kalman_xa));

		kalman_x_err = kalman_x_err + kalman_q;
		kalman_x_err = (1 - kalman_gain) * (kalman_x_err);

		kalman_xa_err = (kalman_xa_err + kalman_qa);
		kalman_xa_err = (1 - kalman_gain_a) * (kalman_xa_err);

		kalman_last_x = kalman_x;
		kalman_last_measured_x = read_bmp_altitude;*/

		total_bmp_altitude -= bmp_over_time[bmp_reading_index];
		bmp_over_time[bmp_reading_index] = read_bmp_altitude;
		total_bmp_altitude += bmp_over_time[bmp_reading_index];

		fast_bmp_altitude = (total_bmp_altitude / 4.00);
		slow_bmp_altitude = (slow_bmp_altitude * 0.900) + (fast_bmp_altitude * 0.100);

		pressure_difference = slow_bmp_altitude - fast_bmp_altitude;

		if(pressure_difference > 0.400)
			pressure_difference = 0.400;
		else if(pressure_difference < -0.400)
			pressure_difference = -0.400;

		if(pressure_difference > 0.120 || pressure_difference < -0.120)
			slow_bmp_altitude -= pressure_difference / 2.00;

		bmp_reading_index++;

		if(bmp_reading_index == 4)
			bmp_reading_index = 0;
	}

	//current_state_double = round(current_state[0] * 100.00) / 100.00;
	//kalman_gain_double = round(kalman_gain_matrix[0] * 100.00) / 100.00;
	//current_state_float = (float)current_state_double;
	//kalman_gain_float = (float)kalman_gain_double;

	last_state[0] = current_state[0];
	last_state[1] = current_state[1];
}

matrix Multiple_Matrix(matrix *a, matrix *b)
{
	matrix mat;
	mat.row = a->row;
	mat.col = b->col;

	for(int i = 0; i < mat.row; i++)
	{
		for(int j = 0; j < mat.col; j++)
		{
			mat.data[j + i * mat.col] = 0;
			for(int k = 0; k < b->row; k++)
			{
				mat.data[j + i * mat.col] += b->data[k * b->col + j] * a->data[k + a->col * i];
			}
		}
	}

	return mat;
}

void Calculate_Altitude_PID()
{
	pid_error_temp = pid_current_altitude_setpoint - slow_bmp_altitude;
	pid_alt_i += ki_alt * pid_error_temp;

	kp_alt_actual = kp_alt;

	if (pid_error_temp > 1.60 || pid_error_temp < -1.60)
	{
		kp_alt_actual = kp_alt * 2.5;
	}

	pid_altitude_over_time_total -= pid_altitude_over_time[pid_altitude_over_time_reading_index];
	pid_altitude_over_time[pid_altitude_over_time_reading_index] = (pid_error_temp - pid_alt_last_error) * kd_alt;
	pid_altitude_over_time_total += pid_altitude_over_time[pid_altitude_over_time_reading_index];

	pid_altitude_over_time_reading_index++;

	if(pid_altitude_over_time_reading_index == 20)
		pid_altitude_over_time_reading_index = 0;

	if(pid_alt_i > 50)
		pid_alt_i = 50;
	else if(pid_alt_i < -50)
		pid_alt_i = -50;

	altitude_pid_output = (int32_t)((pid_error_temp * kp_alt_actual) + pid_alt_i + pid_altitude_over_time_total);

	pid_alt_last_error = pid_error_temp;

	if(altitude_pid_output > 110)
		altitude_pid_output = 110;
	else if(altitude_pid_output < -110)
		altitude_pid_output = -110;
}

int32_t lat_mem[200], lon_mem[200];
float alt_mem[200];

uint8_t sat_count;
int32_t raw_gps_lat, raw_gps_lon;
uint8_t new_gps_data = 0;
int32_t last_raw_gps_lat, last_raw_gps_lon;

//Instantaneous setpoint
int32_t current_lat_setpoint, current_lon_setpoint;
uint8_t gps_setpoint = 0;	//Index to array setpoint

float calculated_lat_error, calculated_lon_error;
float lat_add, lon_add;

float lat_error_over_time_total, lon_error_over_time_total;
float lat_error_over_time[50], lon_error_over_time[50];
uint8_t gps_error_over_time_reading_index = 0;

float last_calculated_lat_error, last_calculated_lon_error;

float kp_gps = 0, kp_gps_actual = 0, ki_gps = 0, kd_gps = 0;
float pid_gps_i = 0;

int32_t gps_roll_modifier, gps_pitch_modifier;
float gps_roll_modifier_north, gps_pitch_modifier_north;

void GPS_PID()
{
	lat_error_over_time_total -= lat_error_over_time[gps_error_over_time_reading_index];
	lat_error_over_time[gps_error_over_time_reading_index] = (calculated_lat_error - last_calculated_lat_error);
	lat_error_over_time_total += lat_error_over_time[gps_error_over_time_reading_index];

	lon_error_over_time_total -= lon_error_over_time[gps_error_over_time_reading_index];
	lon_error_over_time[gps_error_over_time_reading_index] = (calculated_lon_error - last_calculated_lon_error);
	lon_error_over_time_total += lon_error_over_time[gps_error_over_time_reading_index];

	last_calculated_lat_error = calculated_lat_error;
	last_calculated_lon_error = calculated_lon_error;

	gps_error_over_time_reading_index++;

	if(gps_error_over_time_reading_index == 40)
		gps_error_over_time_reading_index = 0;

	pid_error_temp = calculated_lon_error;
	gps_roll_modifier_north = ((pid_error_temp * kp_gps_actual) + (lon_error_over_time_total * kd_gps));

	pid_error_temp = calculated_lat_error;
	gps_pitch_modifier_north = ((pid_error_temp * kp_gps_actual) + (lat_error_over_time_total * kd_gps));

	gps_roll_modifier = (int32_t)((gps_roll_modifier_north * cos(gyro_z_angle * 0.017453)) + (gps_pitch_modifier_north * sin(gyro_z_angle * 0.017453)));
	gps_pitch_modifier = (int32_t)((gps_pitch_modifier_north * cos(gyro_z_angle * 0.017453)) + (gps_roll_modifier_north * sin(gyro_z_angle * -0.017453)));

	if(gps_roll_modifier > 400)
		gps_roll_modifier = 400;
	else if(gps_roll_modifier < -400)
		gps_roll_modifier = -400;

	if(gps_pitch_modifier > 400)
		gps_pitch_modifier = 400;
	else if(gps_pitch_modifier < -400)
		gps_pitch_modifier = -400;
}

float camera_roll_modifier, camera_pitch_modifier;

float kp_camera = 7, kp_camera_actual = 0, ki_camera = 0, kd_camera = 0;
float pid_camera_x_i = 0, pid_camera_y_i;

float last_camera_roll_error = 0, last_camera_pitch_error = 0;

void OpticalFlow_PID()
{
	//Roll
	pid_error_temp = 0 - camera_velocity_y;

	camera_roll_modifier = (pid_error_temp * kp_camera) + ((pid_error_temp - last_camera_roll_error) * kd_camera);

	last_camera_roll_error = pid_error_temp;

	//Pitch
	pid_error_temp = camera_velocity_x - 0;

	camera_pitch_modifier = (pid_error_temp * kp_camera) + ((pid_error_temp - last_camera_pitch_error) * kd_camera);

	last_camera_pitch_error = pid_error_temp;
}

void Accelerometer_Stabilization_PID()
{

}

float z_acc_fast[30], z_acc_slow[30], z_acc_fast_total, z_acc_slow_total;
float hover_throttle = 125;
int32_t idle_throttle = 125;
uint8_t z_acc_fast_reading_index = 0, z_acc_slow_reading_index = 0;
uint32_t launch_timer;

void Launch_Behavior()
{
	z_acc_fast_total -= z_acc_fast[z_acc_fast_reading_index];
	z_acc_fast[z_acc_fast_reading_index] = acc_magnitude;
	z_acc_fast_total += z_acc_fast[z_acc_fast_reading_index];

	z_acc_fast_reading_index++;

	if(z_acc_fast_reading_index == 25)
	{
		z_acc_fast_reading_index = 0;
	}

	if(GetMillisDifference(&launch_timer) >= 1000)
	{
		hover_throttle += 0.08;
		idle_throttle = (int32_t)hover_throttle;

		if((z_acc_fast_total / 25) - acc_magnitude_at_start > 1500)
		{
			//Launched
			launched = 1;
			launching = 0;
			landing = 0;

			pid_current_altitude_setpoint = 3;
			pid_altitude_setpoint = 3;

			lat_mem[0] = raw_gps_lat;
			lon_mem[0] = raw_gps_lon;

			gps_setpoint = 0;
			current_lat_setpoint = lat_mem[0];
			current_lon_setpoint = lon_mem[0];

			ready_for_next_command = 1;
			ready_for_next_command_high_priority = 1;
			altitude_hold_flag = 1;
			gps_hold_flag = 1;
			last_gps_hold_flag = 0;
			gps_waypoint_flag = 0;

			ClearPrintBuffer();
			sprintf((char *)print_text_buffer, "%s%ld%s", "Launched: ", idle_throttle, "\n");
			PrintManualPacket();
		}
	}
}

float temp_max_acc = 0;

void Land_Behavior()
{
	z_acc_fast_total -= z_acc_fast[z_acc_fast_reading_index];
	z_acc_fast[z_acc_fast_reading_index] = acc_magnitude;
	z_acc_fast_total += z_acc_fast[z_acc_fast_reading_index];

	z_acc_fast_reading_index++;

	if(z_acc_fast_reading_index == 25)
	{
		z_acc_fast_reading_index = 0;
	}

	if(GetMillisDifference(&launch_timer) >= 1000)
	{
		pid_altitude_setpoint -= 0.002;

		if(abs((z_acc_fast_total / 25) - acc_magnitude_at_start) > temp_max_acc)
		{
			temp_max_acc = abs((z_acc_fast_total / 25) - acc_magnitude_at_start);
		}

		if((z_acc_fast_total / 25) - acc_magnitude_at_start > 4000)
		{
			//Landed
			launched = 0;
			launching = 0;
			landing = 0;

			pid_current_altitude_setpoint = 3;
			pid_altitude_setpoint = 3;

			hover_throttle = 125;
			idle_throttle = 125;
			altitude_hold_flag = 0;
			gps_hold_flag = 0;
			last_gps_hold_flag = 0;
			gps_waypoint_flag = 0;
			ready_for_next_command = 1;
			ready_for_next_command_high_priority = 1;

			ClearPrintBuffer();
			//sprintf((char *)print_text_buffer, "%s%ld%s", "Landed: ", (int32_t)temp_max_acc, "\n");
			sprintf((char *)print_text_buffer, "%s", "Landed.\n");
			PrintManualPacket();
		}
	}
}
