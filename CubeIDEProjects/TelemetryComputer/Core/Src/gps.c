/*
 * gps.c
 *
 *  Created on: Apr 24, 2021
 *      Author: Nate
 */

#include "gps.h"
#include "main.h"
#include "usart.h"
#include "nrf24radio.h"

uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};

uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};

uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1};

uint8_t gps_buffer[99];
volatile uint8_t gps_buffer_index = 100;
volatile uint8_t new_gps_line = 0;

uint8_t gps_fix = 0;
uint8_t sat_count = 0;
int32_t latitude = 0;
int32_t longitude = 0;

uint8_t waiting_for_gps_packet_sent = 0;

void GPS_Read()
{
	if(new_gps_line)
	{
		new_gps_line = 0;

		//HAL_UART_Transmit(&huart1, gps_buffer, gps_buffer_index, HAL_MAX_DELAY);

		if(gps_buffer[3] == 'L' && gps_buffer[4] == 'L' && gps_buffer[6] == ',')	//No GPS fix
		{
			sat_count = 0;

			if(gps_fix)
			{
				GPS_Send_Packet();
			}

			gps_fix = 0;
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		}
		else if(gps_buffer[3] == 'L' && gps_buffer[4] == 'L')						//GPS Fix
		{
			gps_fix = 1;
		}

		if(gps_fix)
		{
			if(gps_buffer[3] == 'G' && gps_buffer[4] == 'A' && (gps_buffer[43] == '1' || gps_buffer[43] == '2'))
			{
				latitude = ((int32_t)gps_buffer[18] - 48) * 10000000;
				latitude += ((int32_t)gps_buffer[19] - 48) * 1000000;
				latitude += ((int32_t)gps_buffer[21] - 48) * 100000;
				latitude += ((int32_t)gps_buffer[22] - 48) * 10000;
				latitude += ((int32_t)gps_buffer[23] - 48) * 1000;
				latitude += ((int32_t)gps_buffer[24] - 48) * 100;
				latitude += ((int32_t)gps_buffer[25] - 48) * 10;
				latitude /= (int32_t)6;
				latitude += ((int32_t)gps_buffer[16] - 48) * 100000000;
				latitude += ((int32_t)gps_buffer[17] - 48) * 10000000;
				latitude /= (int32_t)10;

				longitude = ((int32_t)gps_buffer[32] - 48) * 10000000;
				longitude += ((int32_t)gps_buffer[33] - 48) * 1000000;
				longitude += ((int32_t)gps_buffer[35] - 48) * 100000;
				longitude += ((int32_t)gps_buffer[36] - 48) * 10000;
				longitude += ((int32_t)gps_buffer[37] - 48) * 1000;
				longitude += ((int32_t)gps_buffer[38] - 48) * 100;
				longitude += ((int32_t)gps_buffer[39] - 48) * 10;
				longitude /= (int32_t)6;
				longitude += ((int32_t)gps_buffer[29] - 48) * 1000000000;
				longitude += ((int32_t)gps_buffer[30] - 48) * 100000000;
				longitude += ((int32_t)gps_buffer[31] - 48) * 10000000;
				longitude /= (int32_t)10;

				if (gps_buffer[27] == 'S')
					latitude *= -1;

				if (gps_buffer[41] == 'W')
					longitude *= -1;

				sat_count = (uint8_t)((int32_t)gps_buffer[45] - 48) * (uint8_t)10;
				sat_count += (uint8_t)((int32_t)gps_buffer[46] - 48);

				GPS_Send_Packet();
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			}
		}

		for(int32_t i = 0; i < 99; i++)
		{
			gps_buffer[i] = '-';
		}

		gps_buffer_index = 0;

		USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE;
	}
}

void GPS_Send_Packet()
{
	if(!waiting_for_gps_packet_sent)
	{
		packets_to_receive[packets_to_receive_counter].payload[0] = 0xFD;
		packets_to_receive[packets_to_receive_counter].payload[1] = sat_count;

		//packets_to_receive[packets_to_receive_counter].width = packet_width;
		//packets_to_receive[packets_to_receive_counter].reliable = 1;

		for(int i = 0; i < 4; i++)
		{
			packets_to_receive[packets_to_receive_counter].payload[i + 2] = *((uint8_t *)&latitude + i);
			packets_to_receive[packets_to_receive_counter].payload[i + 6] = *((uint8_t *)&longitude + i);
		}

		waiting_for_gps_packet_sent = 1;

		if(packets_to_receive_counter < 30)
			packets_to_receive_counter++;
	}
}

void GPS_Init()
{
	HAL_Delay(250);

	for(int32_t i = 0; i < 99; i++)
	{
		gps_buffer[i] = '-';
	}

	Custom_GPS_Send(Disable_GPGSV, 11);
	//HAL_UART_Transmit(&huart2, Disable_GPGSV, 11, HAL_MAX_DELAY);
	HAL_Delay(250);
	Custom_GPS_Send(Set_to_5Hz, 14);
	//HAL_UART_Transmit(&huart2, Set_to_5Hz, 14, HAL_MAX_DELAY);
	HAL_Delay(250);
	Custom_GPS_Send(Set_to_57kbps, 28);
	//HAL_UART_Transmit(&huart2, Set_to_57kbps, 28, HAL_MAX_DELAY);
	HAL_Delay(250);
	//uint8_t temp_uart_buffer[32];
	//sprintf((char *)temp_uart_buffer, "%s", "Worked\n");
	//HAL_UART_Transmit(&huart1, temp_uart_buffer, 7, HAL_MAX_DELAY);
	//USART2->CR1 &= ~(USART_CR1_UE);
	//USART2->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), 57600);
	//USART2->BRR = 57600;
	HAL_UART_Abort(&huart2);
	//HAL_UART_
	HAL_Delay(250);
	huart2.Init.BaudRate = 57600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}

	USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE;
}

void Custom_GPS_Send(uint8_t *data, uint8_t size)
{
	uint8_t bytes_left = size;

	USART2->CR1 |= USART_CR1_TE;

	while(bytes_left > 0)
	{
		USART2->DR = (uint8_t)(*data++);

		while((USART2->SR & USART_SR_TXE) == 0);
		bytes_left--;
	}

	while((USART2->SR & USART_SR_TC) == 0);

	USART2->CR1 &= ~(USART_CR1_TE);
}
