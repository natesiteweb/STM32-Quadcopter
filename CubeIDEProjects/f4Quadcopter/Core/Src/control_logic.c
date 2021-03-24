/*
 * control_logic.c
 *
 *  Created on: Mar 2, 2021
 *      Author: Nate
 */


#include "telemetry.h"
#include "control_logic.h"
#include "main.h"
#include "gpio.h"
#include "string.h"
#include "imu.h"
#include "stdlib.h"
#include "math.h"
#include "bmp280.h"
#include "compass.h"

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

	gyro_x_angle = (gyro_x_angle * 0.9985) + (acc_x * (1.0000 - 0.9985));
	gyro_y_angle = (gyro_y_angle * 0.9985) + (acc_y * (1.0000 - 0.9985));

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

	pid_roll_setpoint -= (gyro_x_angle * 10);
	pid_pitch_setpoint -= (gyro_y_angle * 10);

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

float pid_altitude_setpoint = 2;

float pid_alt_last_error = 0;
int32_t altitude_pid_output;

float kp_alt = 0, ki_alt = 0, kd_alt = 0;
float pid_alt_i = 0;

void Calculate_Altitude_PID()
{
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

	pid_error_temp = pid_altitude_setpoint - slow_bmp_altitude;
	pid_alt_i += ki_alt * pid_error_temp;

	if(pid_alt_i > 100)
		pid_alt_i = 100;
	else if(pid_alt_i < -100)
		pid_alt_i = -100;

	altitude_pid_output = (pid_error_temp * kp_alt) + pid_alt_i + ((pid_error_temp - pid_alt_last_error) * kd_alt);

	if(altitude_pid_output > 110)
		altitude_pid_output = 110;
	else if(altitude_pid_output < -110)
		altitude_pid_output = -110;
}

typedef struct
{
	/*
	 * 0x00: launching
	 * 0x01: landing
	 * 0x02: altitude hold
	 */
	uint16_t behavior_reg1;
	float alt_setpoint;
	int32_t lat_setpoint;
	int32_t lon_setpoint;
} control_register_struct;

control_register_struct control_registers;

uint8_t *direct_var_access_buffer[256];		//Used to access permanent variables(altitude, launch state, etc.)
uint8_t program_memory_buffer[512];			//Virtual memory buffer for storing program variables
uint8_t program_buffer[512];				//Program
uint16_t program_counter = 0;

uint8_t high_priority_program_buffer[32];	//Direct command messages from telemetry
uint8_t high_priority_program_counter = 0;
uint8_t high_priority_program_width = 0;

uint32_t control_loop_wait_timer;
uint32_t control_loop_wait_time = 0;

uint32_t high_priority_control_loop_wait_timer;
uint32_t high_priority_control_loop_wait_time = 0;

int32_t desired_flight_mode;
uint8_t launched = 0, launching = 0, landing = 0;
uint8_t ready_for_next_command = 1, ready_for_next_command_high_priority = 1;
uint8_t manual_mode = 0;

void Control_Loop()
{
	if(high_priority_program_width > 0 && ready_for_next_command_high_priority)
	{
		if(GetMillisDifference(&high_priority_control_loop_wait_timer) >= high_priority_control_loop_wait_time)
		{
			uint16_t increment_index = Parse_Command((uint8_t *)&high_priority_program_buffer, high_priority_program_counter, 1);
			high_priority_program_width -= increment_index;
			high_priority_program_counter += increment_index;
		}
	}
	else if(program_counter < 512 && ready_for_next_command)
	{
		if(GetMillisDifference(&control_loop_wait_timer) >= control_loop_wait_time)
		{
			uint16_t increment_index = Parse_Command((uint8_t *)&program_buffer, program_counter, 0);
			program_counter += increment_index;
		}
	}
}

uint16_t Parse_Command(uint8_t *cmd_array, uint16_t cmd_index, uint8_t high_priority)
{
	uint16_t output_index = 0;

	switch(cmd_array[cmd_index])
	{
	case 0x00:	//No OP
		output_index++;
		break;
	case 0x01:	//Toggle LED: uint8_t
		if(cmd_array[cmd_index + 1] == 0x01)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		}
		else if(cmd_array[cmd_index + 1] == 0x02)
		{
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		}

		output_index += 2;
		break;
	case 0x02:	//Wait: uint32_t
		if(high_priority)
		{
			high_priority_control_loop_wait_timer = GetMillis();
			high_priority_control_loop_wait_time = *((uint32_t *)&cmd_array[cmd_index + 1]);
		}
		else
		{
			control_loop_wait_timer = GetMillis();
			control_loop_wait_time = *((uint32_t *)&cmd_array[cmd_index + 1]);
		}
		output_index += 5;
		break;
	case 0x03:	//Restart program
		output_index = 0;
		break;
	case 0x04:	//Launch
		Parse_Requested_State(LAUNCHED);
		output_index++;
		break;
	case 0x05:	//Land
		Parse_Requested_State(LANDED);
		output_index++;
		break;
	}

	return output_index;
}

void Parse_Requested_State(int32_t requested_state)
{
	switch(requested_state)
	{
	case LANDED:
		if(launched && !landing)
		{
			landing = 1;
			ready_for_next_command = 0;
			ready_for_next_command_high_priority = 0;
			launch_timer = GetMillis();
			ClearPrintBuffer();
			sprintf((char *)print_text_buffer, "%s", "Landing...\n");
			PrintManualPacket();
		}
		break;
	case LAUNCHED:
		if(!launched && !launching)
		{
			launching = 1;
			acc_magnitude_at_start = acc_magnitude;
			ready_for_next_command = 0;
			ready_for_next_command_high_priority = 0;
			launch_timer = GetMillis();
			ClearPrintBuffer();
			sprintf((char *)print_text_buffer, "%s", "Launching...\n");
			PrintManualPacket();
		}
		break;
	}
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
		hover_throttle += 0.0625;
		idle_throttle = (int32_t)hover_throttle;

		if((z_acc_fast_total / 25) - acc_magnitude_at_start > 600)
		{
			//Launched
			launched = 1;
			launching = 0;
			landing = 0;

			ready_for_next_command = 1;
			ready_for_next_command_high_priority = 1;
			altitude_hold_flag = 1;

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
		pid_altitude_setpoint -= 0.003;

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

			hover_throttle = 125;
			idle_throttle = 125;
			altitude_hold_flag = 0;
			ready_for_next_command = 1;
			ready_for_next_command_high_priority = 1;

			ClearPrintBuffer();
			//sprintf((char *)print_text_buffer, "%s%ld%s", "Landed: ", (int32_t)temp_max_acc, "\n");
			sprintf((char *)print_text_buffer, "%s", "Landed.\n");
			PrintManualPacket();
		}
	}
}
