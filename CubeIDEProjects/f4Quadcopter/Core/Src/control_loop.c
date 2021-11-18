/*
 * control_loop.c
 *
 *  Created on: Apr 25, 2021
 *      Author: Nate
 */

#include "telemetry.h"
#include "control_logic.h"
#include "control_loop.h"
#include "main.h"
#include "gpio.h"
//#include "string.h"
#include "imu.h"
//#include "stdlib.h"
#include "math.h"
#include "bmp280.h"
#include "compass.h"

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

direct_access_var direct_access_variables[256];		//Used to access permanent variables(altitude, launch state, etc.)
uint8_t program_memory_buffer[512];					//Virtual memory buffer for storing program variables
uint8_t program_buffer[512];						//Program
uint16_t program_counter = 0;

uint8_t high_priority_program_buffer[32];			//Direct command messages from telemetry
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
	case 0x02:	//Wait millis: uint32_t
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
	case 0x06:	//Fly to single waypoint (not continuous)
		if(!launched)
		{
			TryToLaunchDependency(high_priority);
		}
		else
		{
			gps_setpoint = &cmd_array[cmd_index + 1];

			gps_hold_flag = 1;
			gps_waypoint_flag = 0;
			output_index += 2;
		}

		break;
	case 0x07:	//Fly waypoint range (continuous)
		break;
	case 0x08:	//Return to home
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

void TryToLaunchDependency(uint8_t high_priority)
{
	Parse_Requested_State(LAUNCHED);

	if(high_priority)
	{
		high_priority_control_loop_wait_timer = GetMillis();
		high_priority_control_loop_wait_time = 1000;
	}
	else
	{
		control_loop_wait_timer = GetMillis();
		control_loop_wait_time = 1000;
	}
}
