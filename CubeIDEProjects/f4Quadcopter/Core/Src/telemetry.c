/*
 * telemetry.c
 *
 *  Created on: Feb 28, 2021
 *      Author: Nate
 */

#include "telemetry.h"
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "string.h"
#include "imu.h"
#include "stdlib.h"
#include "control_logic.h"
#include "control_loop.h"
#include "eeprom.h"
#include "bmp280.h"
#include "compass.h"
#include "math.h"

uint32_t telem_min_transmit_timer;
char print_text_buffer[32];
char dynamic_variable_buffer[32];
data_packet manual_packet_buffer[32];
uint8_t manual_packet_count = 0;

data_packet_pointer auto_packet_buffer[32];
uint8_t auto_packet_count = 0;
uint8_t auto_packet_counter = 0;

volatile uint8_t telem_send_buffer[35];
volatile uint8_t telem_receive_buffer[35];
uint8_t telem_receive_read_index = 0;
data_packet empty_data_packet;

volatile uint8_t ack_rate_counter = 0;
uint8_t ack_rate = 10;					//Every x ticks of the radio ask for data

uint8_t transmit_fail_flag = 0;			//If transmission failed, reset everything and try again
uint8_t receive_fail_flag = 0;
volatile uint8_t waiting_to_rx = 0; 	//Waiting until we ask for receive
volatile uint8_t rx_done = 0;			//Response received
volatile uint8_t tx_done = 0;
uint8_t camera_telem = 0;
uint8_t camera_telem_counter = 0;
uint32_t camera_telem_timeout_timer;
uint8_t last_tx_type = 0;				//1: auto packet, 2: manual packet

uint32_t acks_per_second_timer;
uint32_t acks_counted;
uint32_t acks_per_second;

uint32_t time_to_telem_timer, time_to_telem;

uint8_t modify_var_varcount;
uint8_t read_var_varcount;
uint8_t current_var;					//256 variables max
uint8_t current_var_width;

float temp_max_gyro_x = 0;
float temp_max_gyro_y = 0;

float coefficient_x = 0;
float coefficient_y = 0;

void telem_loop()
{
	if(GetMillisDifference(&acks_per_second_timer) >= 1000)
	{
		acks_per_second_timer = GetMillis();
		acks_per_second = acks_counted;
		acks_counted = 0;
	}

	if(camera_telem && GetMicrosDifference(&camera_telem_timeout_timer) >= 4000)
	{
		camera_telem = 0;
	}

	if(HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY)
	{
		if(rx_done)
		{
			rx_done = 0;
			ack_rate_counter = 0;

			if(camera_telem)
			{
				camera_telem = 0;

				if(optical_flow_flag)
				{
					last_camera_x_velocity = raw_camera_x_velocity;
					last_camera_y_velocity = raw_camera_y_velocity;

					raw_camera_x_velocity = *(int32_t *)(((uint8_t *)camera_receive_buf) + 1);
					raw_camera_y_velocity = *(int32_t *)(((uint8_t *)camera_receive_buf) + 5);
					camera_framerate = *(int32_t *)(((uint8_t *)camera_receive_buf) + 9);

					//if(abs(raw_camera_x_velocity - last_camera_x_velocity) > 6)
					//	raw_camera_x_velocity = last_camera_x_velocity;

					//if(abs(raw_camera_y_velocity - last_camera_y_velocity) > 6)
					//	raw_camera_y_velocity = last_camera_y_velocity;

					float x_from_angle = 0;
					float y_from_angle = 0;

					if(cos(((float)gyro_y * 0.016023) * 0.033) != 0 && abs(raw_camera_x_velocity) < 2)
						x_from_angle = tan(((float)gyro_y * 0.016023) * 0.033);

					if(cos(((float)gyro_x * 0.003237) * 0.033) != 0 && abs(raw_camera_y_velocity) < 2)
						y_from_angle = tan(((float)gyro_x * 0.003237) * 0.033);

					float camera_distance = slow_bmp_altitude;

					if(camera_distance < 0)
						camera_distance *= -1;

					camera_velocity_x = (tan((float)(raw_camera_x_velocity) * 0.004013) - x_from_angle) * (camera_distance * 100);
					camera_velocity_y = (tan((float)(raw_camera_y_velocity) * 0.005662) + y_from_angle) * (camera_distance * 100);

					camera_displacement_x += camera_velocity_x;
					camera_displacement_y += camera_velocity_y;

					if(gyro_y > temp_max_gyro_y)
					{
						temp_max_gyro_y = gyro_y;

						coefficient_x = ((((float)raw_camera_x_velocity) * 0.004013) / (gyro_y * 0.033)) * 1000;

						/*EEPROM_Clear_Buffer();
						eeprom_write_buffer_width = 2;
						EEPROM_Write_Buffer((uint8_t *)&coefficient_x, 4);
						EEPROM_Write_Buffer((uint8_t *)&coefficient_y, 4);
						EEPROM_Save_Page(96);*/
						SendDynamicVariable("Coef X", 3, (uint8_t *)&coefficient_x);
					}

					if(gyro_x > temp_max_gyro_x)
					{
						temp_max_gyro_x = gyro_x;

						coefficient_y = ((((float)raw_camera_y_velocity) * 0.005662) / (gyro_x * -0.033)) * 1000;

						/*EEPROM_Clear_Buffer();
						eeprom_write_buffer_width = 2;
						EEPROM_Write_Buffer((uint8_t *)&coefficient_x, 4);
						EEPROM_Write_Buffer((uint8_t *)&coefficient_y, 4);
						EEPROM_Save_Page(96);*/
						SendDynamicVariable("Coef Y", 3, (uint8_t *)&coefficient_y);
					}

					new_camera_data = 1;

					//if(abs(camera_displacement_x) > 20)
					//	camera_displacement_x = last_camera_displacement_x;

					//if(abs(camera_displacement_y) > 20)
					//	camera_displacement_y = last_camera_displacement_y;

					SendDynamicVariable("Cam X", 3, (uint8_t *)&camera_displacement_x);
					SendDynamicVariable("Cam Y", 3, (uint8_t *)&camera_displacement_y);
					SendDynamicVariable("FPS", 2, (uint8_t *)&camera_framerate);
				}
			}
			else
			{
				switch(telem_receive_buffer[0])
				{
				case 0x00:
					break;
				case CALIBRATE_GYRO_REQUEST:
					if(!launched)
					{
						Calibrate_BMP280();
						Calibrate_IMU();
						ClearPrintBuffer();
						sprintf((char *)print_text_buffer, "%s", "Gyro Calibrated.\n");
						PrintManualPacket();
					}
					break;
				case PID_GAIN_FIRST_REQUEST:
					ClearManualBuffer();
					manual_packet_buffer[manual_packet_count].width = 1;
					manual_packet_buffer[manual_packet_count].reliable = 1;
					AddIDToManualBuffer(PID_GAIN_FIRST_PACKET);
					AddToManualBuffer((uint8_t *)&kp_roll, 4);
					AddToManualBuffer((uint8_t *)&ki_roll, 4);
					AddToManualBuffer((uint8_t *)&kd_roll, 4);
					AddToManualBuffer((uint8_t *)&kp_yaw, 4);
					AddToManualBuffer((uint8_t *)&ki_yaw, 4);
					AddToManualBuffer((uint8_t *)&kd_yaw, 4);

					if(manual_packet_count < 31)
						manual_packet_count++;
					break;
				case PID_GAIN_SECOND_REQUEST:
					ClearManualBuffer();
					manual_packet_buffer[manual_packet_count].width = 1;
					manual_packet_buffer[manual_packet_count].reliable = 1;
					AddIDToManualBuffer(PID_GAIN_SECOND_PACKET);
					AddToManualBuffer((uint8_t *)&kp_alt, 4);
					AddToManualBuffer((uint8_t *)&ki_alt, 4);
					AddToManualBuffer((uint8_t *)&kd_alt, 4);
					AddToManualBuffer((uint8_t *)&kp_gps, 4);
					AddToManualBuffer((uint8_t *)&ki_gps, 4);
					AddToManualBuffer((uint8_t *)&kd_gps, 4);

					if(manual_packet_count < 31)
						manual_packet_count++;
					break;
				case PID_GAIN_FIRST_UPDATE_REQUEST:
					telem_receive_read_index = 1;
					ReadReceiveBuffer((uint8_t *)&kp_roll, 4);
					ReadReceiveBuffer((uint8_t *)&ki_roll, 4);
					ReadReceiveBuffer((uint8_t *)&kd_roll, 4);
					ReadReceiveBuffer((uint8_t *)&kp_yaw, 4);
					ReadReceiveBuffer((uint8_t *)&ki_yaw, 4);
					ReadReceiveBuffer((uint8_t *)&kd_yaw, 4);

					EEPROM_Clear_Buffer();
					eeprom_write_buffer_width = 2;
					EEPROM_Write_Buffer((uint8_t *)&kp_roll, 4);
					EEPROM_Write_Buffer((uint8_t *)&ki_roll, 4);
					EEPROM_Write_Buffer((uint8_t *)&kd_roll, 4);
					EEPROM_Write_Buffer((uint8_t *)&kp_yaw, 4);
					EEPROM_Write_Buffer((uint8_t *)&ki_yaw, 4);
					EEPROM_Write_Buffer((uint8_t *)&kd_yaw, 4);
					EEPROM_Save_Page(0);

					ClearManualBuffer();
					manual_packet_buffer[manual_packet_count].width = 1;
					manual_packet_buffer[manual_packet_count].reliable = 1;
					AddIDToManualBuffer(PID_GAIN_FIRST_PACKET);
					AddToManualBuffer((uint8_t *)&kp_roll, 4);
					AddToManualBuffer((uint8_t *)&ki_roll, 4);
					AddToManualBuffer((uint8_t *)&kd_roll, 4);
					AddToManualBuffer((uint8_t *)&kp_yaw, 4);
					AddToManualBuffer((uint8_t *)&ki_yaw, 4);
					AddToManualBuffer((uint8_t *)&kd_yaw, 4);

					if(manual_packet_count < 31)
						manual_packet_count++;
					break;
				case PID_GAIN_SECOND_UPDATE_REQUEST:
					telem_receive_read_index = 1;
					ReadReceiveBuffer((uint8_t *)&kp_alt, 4);
					ReadReceiveBuffer((uint8_t *)&ki_alt, 4);
					ReadReceiveBuffer((uint8_t *)&kd_alt, 4);
					ReadReceiveBuffer((uint8_t *)&kp_gps, 4);
					ReadReceiveBuffer((uint8_t *)&ki_gps, 4);
					ReadReceiveBuffer((uint8_t *)&kd_gps, 4);

					EEPROM_Clear_Buffer();
					eeprom_write_buffer_width = 2;
					EEPROM_Write_Buffer((uint8_t *)&kp_alt, 4);
					EEPROM_Write_Buffer((uint8_t *)&ki_alt, 4);
					EEPROM_Write_Buffer((uint8_t *)&kd_alt, 4);
					EEPROM_Write_Buffer((uint8_t *)&kp_gps, 4);
					EEPROM_Write_Buffer((uint8_t *)&ki_gps, 4);
					EEPROM_Write_Buffer((uint8_t *)&kd_gps, 4);
					EEPROM_Save_Page(32);

					ClearManualBuffer();
					manual_packet_buffer[manual_packet_count].width = 1;
					manual_packet_buffer[manual_packet_count].reliable = 1;
					AddIDToManualBuffer(PID_GAIN_SECOND_PACKET);
					AddToManualBuffer((uint8_t *)&kp_alt, 4);
					AddToManualBuffer((uint8_t *)&ki_alt, 4);
					AddToManualBuffer((uint8_t *)&kd_alt, 4);
					AddToManualBuffer((uint8_t *)&kp_gps, 4);
					AddToManualBuffer((uint8_t *)&ki_gps, 4);
					AddToManualBuffer((uint8_t *)&kd_gps, 4);

					if(manual_packet_count < 31)
						manual_packet_count++;
					break;
				case CALIBRATE_COMPASS_REQUEST:
					Calibrate_Compass();
					break;
				case DO_CMD_REQUEST:
					telem_receive_read_index = 1;
					ReadReceiveBuffer(&high_priority_program_width, 1);
					ReadReceiveBuffer((uint8_t *)&high_priority_program_buffer, high_priority_program_width);
					high_priority_program_counter = 0;
					break;
				case UPLOAD_CMD_REQUEST:
					break;
				case MODIFY_VARIABLE_REQUEST:
					telem_receive_read_index = 1;
					ReadReceiveBuffer(&modify_var_varcount, 1);

					ClearManualBuffer();
					manual_packet_buffer[manual_packet_count].width = 1;
					manual_packet_buffer[manual_packet_count].reliable = 1;
					AddIDToManualBuffer(READ_VARIABLE_PACKET);
					AddToManualBuffer(&modify_var_varcount, 1);

					for(int i = 0; i < modify_var_varcount; i++)
					{
						ReadReceiveBuffer(&current_var, 1);
						ReadReceiveBuffer(&current_var_width, 1);

						if(!direct_access_variables[current_var].protected)
							ReadReceiveBuffer(direct_access_variables[current_var].var, current_var_width);
						else
							telem_receive_read_index += current_var_width;

						AddToManualBuffer(&direct_access_variables[current_var].var_index, 1);
						AddToManualBuffer(direct_access_variables[current_var].var, direct_access_variables[current_var].width);
					}

					if(manual_packet_count < 31)
						manual_packet_count++;

					break;
				case READ_VARIABLE_REQUEST:
					ClearManualBuffer();
					manual_packet_buffer[manual_packet_count].width = 1;
					manual_packet_buffer[manual_packet_count].reliable = 1;
					AddIDToManualBuffer(READ_VARIABLE_PACKET);

					telem_receive_read_index = 1;
					ReadReceiveBuffer(&read_var_varcount, 1);

					AddToManualBuffer(&read_var_varcount, 1);

					for(int i = 0; i < read_var_varcount; i++)
					{
						ReadReceiveBuffer(&current_var, 1);
						//AddToManualBuffer(&direct_access_variables[current_var].width, 1);
						AddToManualBuffer(&direct_access_variables[current_var].var_index, 1);
						AddToManualBuffer(direct_access_variables[current_var].var, direct_access_variables[current_var].width);
					}

					if(manual_packet_count < 31)
						manual_packet_count++;

					break;
				case GPS_PACKET_UPDATE:
					telem_receive_read_index = 1;
					ReadReceiveBuffer(&sat_count, 1);

					if(sat_count > 2)
					{
						last_raw_gps_lat = raw_gps_lat;
						last_raw_gps_lon = raw_gps_lon;

						ReadReceiveBuffer((uint8_t *)&raw_gps_lat, 4);
						ReadReceiveBuffer((uint8_t *)&raw_gps_lon, 4);

						calculated_lat_error = (float)(raw_gps_lat - current_lat_setpoint);
						calculated_lon_error = (float)(current_lon_setpoint - raw_gps_lon);

						lat_add = (float)(raw_gps_lat - last_raw_gps_lat) / (float)20.000;
						lon_add = (float)(raw_gps_lon - last_raw_gps_lon) / (float)20.000;

						new_gps_data = 1;

						uint8_t var_indexes[3] = {3, 4, 5};

						SendDirectVariablePacket(3, var_indexes);
					}

					break;
				case GPS_MEM_PACKET_UPDATE:
					ClearManualBuffer();
					manual_packet_buffer[manual_packet_count].width = 1;
					manual_packet_buffer[manual_packet_count].reliable = 1;
					AddIDToManualBuffer(GPS_PACKET);

					telem_receive_read_index = 1;
					ReadReceiveBuffer(&modify_var_varcount, 1);	//First index
					ReadReceiveBuffer(&current_var_width, 1);	//Count

					AddToManualBuffer(&modify_var_varcount, 1);
					AddToManualBuffer(&current_var_width, 1);

					for(int i = 0; i < current_var_width; i++)
					{
						//ReadReceiveBuffer(&current_var, 1);
						ReadReceiveBuffer((uint8_t *)&lat_mem[modify_var_varcount + i], 4);
						ReadReceiveBuffer((uint8_t *)&lon_mem[modify_var_varcount + i], 4);
						ReadReceiveBuffer((uint8_t *)&alt_mem[modify_var_varcount + i], 4);

						AddToManualBuffer((uint8_t *)&lat_mem[modify_var_varcount + i], 4);
						AddToManualBuffer((uint8_t *)&lon_mem[modify_var_varcount + i], 4);
						AddToManualBuffer((uint8_t *)&alt_mem[modify_var_varcount + i], 4);
					}

					if(manual_packet_count < 31)
						manual_packet_count++;

					break;
				case GPS_PACKET_REQUEST:	//Send packets to GCS
					ClearManualBuffer();
					manual_packet_buffer[manual_packet_count].width = 1;
					manual_packet_buffer[manual_packet_count].reliable = 1;
					AddIDToManualBuffer(GPS_PACKET);

					telem_receive_read_index = 1;
					ReadReceiveBuffer(&read_var_varcount, 1);//Using this as first index to send
					ReadReceiveBuffer(&current_var_width, 1);//Using this as index count

					AddToManualBuffer(&read_var_varcount, 1);
					AddToManualBuffer(&current_var_width, 1);

					for(int i = 0; i < current_var_width; i++)
					{
						//AddToManualBuffer(&direct_access_variables[current_var].width, 1);
						AddToManualBuffer((uint8_t *)&lat_mem[read_var_varcount + i], 4);
						AddToManualBuffer((uint8_t *)&lon_mem[read_var_varcount + i], 4);
						AddToManualBuffer((uint8_t *)&alt_mem[read_var_varcount + i], 4);
					}

					if(manual_packet_count < 31)
						manual_packet_count++;

					break;
				}
			}
		}

		if(tx_done)
		{
			tx_done = 0;
			if(!camera_telem)
			{
				if(last_tx_type == 2 && manual_packet_count > 0)
				{
					for(int i = 0; i < manual_packet_count - 1; i++)
					{
						for(int j = 0; j < 35; j++)
						{
							manual_packet_buffer[i].payload[j] = manual_packet_buffer[i + 1].payload[j];
						}
					}

					manual_packet_count--;
				}
			}
		}

		if(waiting_to_rx)
		{
			if(!camera_telem)
			{
				if(HAL_I2C_Master_Seq_Receive_IT(&hi2c2, (uint8_t)(0x04 << 1), (uint8_t *)telem_receive_buffer, 34, I2C_LAST_FRAME) != HAL_OK)
				{
					transmit_fail_flag = 1;
					ack_rate_counter = 0;
					waiting_to_rx = 0;
				}
				else
				{
					waiting_to_rx = 0;
					//ack_rate_counter = 0;
				}
			}
			else
			{
				if(HAL_I2C_Master_Seq_Receive_IT(&hi2c2, (uint8_t)(0x58 << 1), (uint8_t *)camera_receive_buf, 32, I2C_LAST_FRAME) != HAL_OK)
				{
					transmit_fail_flag = 1;
					ack_rate_counter = 0;
					waiting_to_rx = 0;
				}
				else
				{
					waiting_to_rx = 0;
					//ack_rate_counter = 0;
				}
			}
		}

		if(!waiting_to_rx && GetMicrosDifference(&telem_min_transmit_timer) >= 2500)
		{
			if(!camera_telem)
			{
				for(int i = 0; i < 35; i++)
				{
					telem_send_buffer[i] = '\0';
				}

				if(manual_packet_count > 0)
				{
					last_tx_type = 2;

					for(int i = 0; i < manual_packet_buffer[0].width; i++)
					{
						telem_send_buffer[i] = manual_packet_buffer[0].payload[i];
					}

					telem_send_buffer[32] = manual_packet_buffer[0].width;
					telem_send_buffer[33] = manual_packet_buffer[0].reliable;//Unreliable
					telem_send_buffer[34] = 0;//No data
				}
				else
				{
					last_tx_type = 1;

					telem_send_buffer[0] = auto_packet_buffer[auto_packet_counter].id;
					uint8_t var_index = 1;

					for(int i = 0; i < auto_packet_buffer[auto_packet_counter].var_count; i++)
					{
						for(int j = 0; j < auto_packet_buffer[auto_packet_counter].width[i]; j++)
						{
							telem_send_buffer[var_index] = *((uint8_t *)(auto_packet_buffer[auto_packet_counter].payload[i]) + j);

							var_index++;
						}
					}

					telem_send_buffer[32] = var_index;
					telem_send_buffer[33] = auto_packet_buffer[auto_packet_counter].reliable;

					auto_packet_counter++;

					if(auto_packet_counter >= auto_packet_count)
						auto_packet_counter = 0;
				}

				camera_telem_counter++;

				if(camera_telem_counter >= 12)
				{
					camera_telem_counter = 0;
					camera_telem = 1;
				}
			}
			else
			{

			}

			//sprintf((char*)send_buffer, "%ld%s", ppm_channels[2], "\r\n");//int32_t
			//sprintf((char*)send_buffer, "%c%c%lu%s%hd%s", 0x09 , 0x1E, GetMillisDifference(&test_millis_timer)/*ppm_channels[2]*/, ":", test_gyro_x, "\r\n");//uint32_t

			//sprintf((char*)send_buffer, "%lu%s%hd%s", GetMillisDifference(&test_millis_timer)/*ppm_channels[2]*/, ":", test_gyro_x, "\r\n");//uint32_t
			//sprintf((char*)send_buffer, "%c%c%lu%s%hd%s", 0x09 , strlen((char*)send_buffer), GetMicrosDifference(&test_millis_timer)/*ppm_channels[2]*/, ":", test_gyro_x, "\r\n");//uint32_t

			//sprintf((char*)telem_send_buffer, "%lu%s%ld%lu%s", how_long_to_loop/*ppm_channels[2]*/, ":", ((int32_t)gyro_x), (uint32_t)abs((gyro_x - ((int32_t)gyro_x)) * 10), "\r\n");//uint32_t
			//sprintf((char*)telem_send_buffer, "%c%c%lu%s%ld%lu%s", 0x09 , strlen((char*)telem_send_buffer), how_long_to_loop/*ppm_channels[2]*/, ":", ((int32_t)gyro_x), (uint32_t)abs((gyro_x - ((int32_t)gyro_x)) * 10), "\r\n");//uint32_t

			telem_send_buffer[33] = 0;//Unreliable
			telem_send_buffer[34] = 0;//No data

			//ack_rate_counter = 0;
			if(!camera_telem)
			{
				if(ack_rate_counter < 0xFF)
					ack_rate_counter++;

				if(ack_rate_counter == ack_rate)
				{
					//telem_send_buffer[34] = 1;

					if(HAL_I2C_Master_Seq_Transmit_IT(&hi2c2, (uint8_t)(0x04 << 1), (uint8_t *)telem_send_buffer, 35, I2C_FIRST_FRAME) != HAL_OK)
					{
						transmit_fail_flag = 1;
						ack_rate_counter = 0;
					}
					else
					{
						waiting_to_rx = 1;
					}
				}
				else
				{
					//telem_send_buffer[34] = 0;

					if(HAL_I2C_Master_Seq_Transmit_IT(&hi2c2, (uint8_t)(0x04 << 1), (uint8_t *)telem_send_buffer, 35, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
					{
						transmit_fail_flag = 1;
						ack_rate_counter = 0;
					}
				}
			}
			else
			{
				if(HAL_I2C_Master_Seq_Transmit_IT(&hi2c2, (uint8_t)(0x58 << 1), (uint8_t *)camera_send_buf, 32, I2C_FIRST_FRAME) != HAL_OK)
				{
					transmit_fail_flag = 1;
					//ack_rate_counter = 0;
				}
				else
				{
					waiting_to_rx = 1;
					camera_telem_timeout_timer = GetMicros();
				}
			}

			telem_min_transmit_timer = GetMicros();

			//HAL_I2C_Master_Transmit_DMA(&hi2c2, (uint8_t)(0x04 << 1), (uint8_t *)telem_send_buffer, 35);
		}
	}
}

void SendDynamicVariable(const char *varName, uint8_t varType, uint8_t *num)
{
	//ClearManualBuffer();
	//sprintf((char*)(manual_packet_buffer[manual_packet_count].payload), "%c%c%c%s", DYNAMIC_VARIABLE_PACKET, varType, strlen((char*)(dynamic_variable_buffer)), dynamic_variable_buffer);
	//manual_packet_buffer[manual_packet_count].width = strlen((char *)(manual_packet_buffer[manual_packet_count].payload));
	//manual_packet_buffer[manual_packet_count].reliable = 1;
	if(manual_packet_count < 31)
	{
		ClearManualBuffer();
		manual_packet_buffer[manual_packet_count].width = 1;
		manual_packet_buffer[manual_packet_count].reliable = 1;
		AddIDToManualBuffer(DYNAMIC_VARIABLE_PACKET);

		uint8_t nameLen = (uint8_t)strlen((char *)varName);

		AddToManualBuffer(&varType, 1);
		AddToManualBuffer(&nameLen, 1);
		if(varType == 0)
			AddToManualBuffer(num, 1);
		else
			AddToManualBuffer(num, 4);

		AddToManualBuffer((uint8_t *)varName, nameLen);

		//if(manual_packet_count < 31)
		manual_packet_count++;
	}
}

void SendDirectVariablePacket(uint8_t varcount, uint8_t *index)
{
	ClearManualBuffer();
	manual_packet_buffer[manual_packet_count].width = 1;
	manual_packet_buffer[manual_packet_count].reliable = 1;
	AddIDToManualBuffer(READ_VARIABLE_PACKET);

	AddToManualBuffer(&varcount, 1);

	for(int i = 0; i < varcount; i++)
	{
		AddToManualBuffer(&direct_access_variables[*(index + (uint8_t)i)].var_index, 1);
		AddToManualBuffer(direct_access_variables[*(index + (uint8_t)i)].var, direct_access_variables[*(index + (uint8_t)i)].width);
	}

	if(manual_packet_count < 31)
		manual_packet_count++;
}

void ClearManualBuffer()
{
	manual_packet_buffer[manual_packet_count].reliable = 0;
	for(int i = 0; i < 35; i++)
	{
		manual_packet_buffer[manual_packet_count].payload[i] = '\0';
	}
}

void ClearPrintBuffer()
{
	for(int i = 0; i < 32; i++)
	{
		print_text_buffer[i] = '\0';
	}
}

void ClearDynamicVariableBuffer()
{
	for(int i = 0; i < 32; i++)
	{
		dynamic_variable_buffer[i] = '\0';
	}
}

//Used to write text to console
void PrintManualPacket()
{
	ClearManualBuffer();

	sprintf((char*)(manual_packet_buffer[manual_packet_count].payload), "%s", print_text_buffer);//uint32_t
	sprintf((char*)(manual_packet_buffer[manual_packet_count].payload), "%c%c%s", 0x09 , strlen((char*)(manual_packet_buffer[manual_packet_count].payload)), print_text_buffer);

	manual_packet_buffer[manual_packet_count].width = strlen((char *)(manual_packet_buffer[manual_packet_count].payload));

	if(manual_packet_count < 31)
		manual_packet_count++;
}

void AddToAutoBuffer(uint8_t buf_index, uint8_t *num, uint8_t size)
{
	auto_packet_buffer[buf_index].payload[auto_packet_buffer[buf_index].var_count] = (uint8_t *)num;
	auto_packet_buffer[buf_index].width[auto_packet_buffer[buf_index].var_count] = size;
	auto_packet_buffer[buf_index].total_width += size;
	auto_packet_buffer[buf_index].var_count += 1;
}

void AddIDToManualBuffer(uint8_t packet_id)
{
	manual_packet_buffer[manual_packet_count].payload[0] = packet_id;
}

void AddToManualBuffer(uint8_t *num, uint8_t size)
{
	for(int i = 0; i < size; i++)
	{
		manual_packet_buffer[manual_packet_count].payload[manual_packet_buffer[manual_packet_count].width + i] = *(((uint8_t *)num) + i);
	}

	manual_packet_buffer[manual_packet_count].width += size;
}

void ReadReceiveBuffer(uint8_t *output, uint8_t size)
{
	for(int i = 0; i < size; i++)
	{
		*(((uint8_t *)output) + i) = telem_receive_buffer[telem_receive_read_index + i];
	}

	telem_receive_read_index += size;
}
