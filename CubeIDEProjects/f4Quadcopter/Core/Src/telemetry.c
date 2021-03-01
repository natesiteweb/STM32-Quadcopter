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

data_packet manual_packet_buffer[32];
uint8_t manual_packet_count = 0;

data_packet_pointer auto_packet_buffer[32];
uint8_t auto_packet_count = 0;
uint8_t auto_packet_counter = 0;

volatile uint8_t telem_send_buffer[35];
data_packet empty_data_packet;
uint8_t telem_receive_buffer[35];

uint32_t telem_receive_timout_timer;

uint8_t receive_telem_flag = 0;
uint8_t new_telem_received = 0;
uint8_t waiting_for_ack = 0;

volatile uint8_t ack_rate_counter = 0;
uint8_t ack_rate = 20;//Every x ticks of the radio ask for data

void telem_loop()
{
	if(HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY && telem_send_buffer[34] == 0)
	{
		for(int i = 0; i < 35; i++)
		{
			telem_send_buffer[i] = '\0';
		}

		if(manual_packet_count > 0)
		{
			//telem_send_buffer = &manual_packet_buffer[0].payload;
			//HAL_I2C_Master_Transmit_DMA(&hi2c2, (uint8_t)(0x04 << 1), (uint8_t *)manual_packet_buffer[0].payload, 35);
		}
		else
		{
			//HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			telem_send_buffer[0] = auto_packet_buffer[auto_packet_counter].id;
			uint8_t var_index = 1;

			for(int i = 0; i < auto_packet_buffer[auto_packet_counter].var_count; i++)
			{
				for(int j = 0; j < auto_packet_buffer[auto_packet_counter].width[i]; j++)
				{
					telem_send_buffer[var_index] = *((uint8_t *)(auto_packet_buffer[auto_packet_counter].payload[i]) + j);

					var_index += 1;
				}
			}

			telem_send_buffer[32] = var_index;

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

		if(ack_rate_counter == ack_rate)
		{
			telem_send_buffer[34] = 1;
			//waiting_for_ack = 1;
		}
		else
		{
			telem_send_buffer[34] = 0;
		}

		test_millis_timer = GetMillis();

		//telem_send_buffer[32] = 30;
		telem_send_buffer[33] = 0;//Unreliable
		//send_buffer[34] = 0;//No data

		HAL_I2C_Master_Transmit_DMA(&hi2c2, (uint8_t)(0x04 << 1), (uint8_t *)telem_send_buffer, 35);
		telem_receive_timout_timer = GetMillis();
	}
}

void AddToAutoBuffer(uint8_t buf_index, uint8_t *num, uint8_t size)
{
	auto_packet_buffer[buf_index].payload[auto_packet_buffer[buf_index].var_count] = (uint8_t *)num;
	auto_packet_buffer[buf_index].width[auto_packet_buffer[buf_index].var_count] = size;
	auto_packet_buffer[buf_index].total_width += size;
	auto_packet_buffer[buf_index].var_count += 1;
}
