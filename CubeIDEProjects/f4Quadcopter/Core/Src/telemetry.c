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
#include "eeprom.h"
#include "bmp280.h"
#include "compass.h"

uint32_t telem_min_transmit_timer;
char print_text_buffer[32];
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
uint8_t ack_rate = 10;//Every x ticks of the radio ask for data

uint8_t transmit_fail_flag = 0;	//If transmission failed, reset everything and try again
uint8_t receive_fail_flag = 0;
volatile uint8_t waiting_to_rx = 0; 	//Waiting until we ask for receive
volatile uint8_t rx_done = 0;			//Response received
volatile uint8_t tx_done = 0;
uint8_t last_tx_type = 0;				//1: auto packet, 2: manual packet

uint32_t acks_per_second_timer;
uint32_t acks_counted;
uint32_t acks_per_second;

uint32_t time_to_telem_timer, time_to_telem;

void telem_loop()
{
	if(GetMillisDifference(&acks_per_second_timer) >= 1000)
	{
		acks_per_second_timer = GetMillis();
		acks_per_second = acks_counted;
		acks_counted = 0;
	}

	if(HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY)
	{
		if(rx_done)
		{
			rx_done = 0;
			ack_rate_counter = 0;

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

				EEPROM_Clear_Buffer();
				eeprom_write_buffer_width = 2;
				EEPROM_Write_Buffer((uint8_t *)&kp_alt, 4);
				EEPROM_Write_Buffer((uint8_t *)&ki_alt, 4);
				EEPROM_Write_Buffer((uint8_t *)&kd_alt, 4);
				EEPROM_Save_Page(32);

				ClearManualBuffer();
				manual_packet_buffer[manual_packet_count].width = 1;
				manual_packet_buffer[manual_packet_count].reliable = 1;
				AddIDToManualBuffer(PID_GAIN_SECOND_PACKET);
				AddToManualBuffer((uint8_t *)&kp_alt, 4);
				AddToManualBuffer((uint8_t *)&ki_alt, 4);
				AddToManualBuffer((uint8_t *)&kd_alt, 4);

				if(manual_packet_count < 31)
					manual_packet_count++;
				break;
			case CALIBRATE_COMPASS_REQUEST:
				Calibrate_Compass();
				break;
			case DO_CMD_PACKET:
				telem_receive_read_index = 1;
				ReadReceiveBuffer(&high_priority_program_width, 1);
				ReadReceiveBuffer((uint8_t *)&high_priority_program_buffer, high_priority_program_width);
				high_priority_program_counter = 0;
				break;
			case UPLOAD_CMD_PACKET:
				break;
			}
		}

		if(tx_done)
		{
			tx_done = 0;

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

		if(waiting_to_rx)
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

		if(!waiting_to_rx && GetMicrosDifference(&telem_min_transmit_timer) >= 2500)
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

			//sprintf((char*)send_buffer, "%ld%s", ppm_channels[2], "\r\n");//int32_t
			//sprintf((char*)send_buffer, "%c%c%lu%s%hd%s", 0x09 , 0x1E, GetMillisDifference(&test_millis_timer)/*ppm_channels[2]*/, ":", test_gyro_x, "\r\n");//uint32_t

			//sprintf((char*)send_buffer, "%lu%s%hd%s", GetMillisDifference(&test_millis_timer)/*ppm_channels[2]*/, ":", test_gyro_x, "\r\n");//uint32_t
			//sprintf((char*)send_buffer, "%c%c%lu%s%hd%s", 0x09 , strlen((char*)send_buffer), GetMicrosDifference(&test_millis_timer)/*ppm_channels[2]*/, ":", test_gyro_x, "\r\n");//uint32_t

			//sprintf((char*)telem_send_buffer, "%lu%s%ld%lu%s", how_long_to_loop/*ppm_channels[2]*/, ":", ((int32_t)gyro_x), (uint32_t)abs((gyro_x - ((int32_t)gyro_x)) * 10), "\r\n");//uint32_t
			//sprintf((char*)telem_send_buffer, "%c%c%lu%s%ld%lu%s", 0x09 , strlen((char*)telem_send_buffer), how_long_to_loop/*ppm_channels[2]*/, ":", ((int32_t)gyro_x), (uint32_t)abs((gyro_x - ((int32_t)gyro_x)) * 10), "\r\n");//uint32_t


			if(ack_rate_counter < 0xFF)
				ack_rate_counter++;

			telem_send_buffer[33] = 0;//Unreliable
			telem_send_buffer[34] = 0;//No data

			//ack_rate_counter = 0;

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

			telem_min_transmit_timer = GetMicros();

			//HAL_I2C_Master_Transmit_DMA(&hi2c2, (uint8_t)(0x04 << 1), (uint8_t *)telem_send_buffer, 35);
		}
	}
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
