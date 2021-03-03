/*
 * nrf24radio.c
 *
 *  Created on: Dec 3, 2020
 *      Author: Nate
 */
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "nrf24radio.h"


/*
 * Packet buffers
 */
//Packets to send to ground control
//Send as little of these as possible
//All packets require ack, but reliable packets are sent again if max retransmissions fails
data_packet reliable_packets_to_gcs[32];
volatile data_packet current_i2c_packet;
data_packet unreliable_packet;
uint8_t reliable_packet_to_gcs_counter = 0;

//Packets to send to flight computer
volatile data_packet packets_to_receive[32];
uint8_t packets_to_receive_counter = 0;

volatile uint8_t radio_irq_flag = 0;
volatile uint8_t waiting_for_ack = 0;

void NRF24_Init(NRF24_RADIO *radio)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

	NRF24_WriteBit(radio, 0, 0, 1); //register#, bit#, and value 0 or 1, ::  0,0,1 RX Mode
	NRF24_WriteBit(radio, 0, 1, 1); //register, bit, and value 0,1,1 PowerUP
	    //NRF24_WriteBit(radio, 0, 4, 1); //RT Mask turns off the RT interrupt
	    //NRF24_WriteBit(radio, 0, 5, 1); //TX Mask turns off the TX interrupt
	    //NRF24_WriteBit(radio, 0, 6, 1);

	NRF24_WriteBit(radio, 4, 4, 0);//1 wait 2750ms for AA
	NRF24_WriteBit(radio, 4, 5, 0);//1
	NRF24_WriteBit(radio, 4, 6, 1);//0
	NRF24_WriteBit(radio, 4, 7, 1);//1

	NRF24_WriteBit(radio, 4, 3, 0);
	NRF24_WriteBit(radio, 4, 2, 1);
	NRF24_WriteBit(radio, 4, 1, 0);
	NRF24_WriteBit(radio, 4, 0, 1);

	NRF24_WriteBit(radio, 6, 3, 0);
	NRF24_WriteBit(radio, 6, 5, 1); //250kbps

	NRF24_WriteBit(radio, 5, 6, 1);
	NRF24_WriteBit(radio, 5, 5, 1);
	NRF24_WriteBit(radio, 5, 4, 0);
	NRF24_WriteBit(radio, 5, 3, 1);
	NRF24_WriteBit(radio, 5, 2, 1);
	NRF24_WriteBit(radio, 5, 1, 1);
	NRF24_WriteBit(radio, 5, 0, 0);

	NRF24_WriteBit(radio, 29, 2, 1);
	NRF24_WriteBit(radio, 29, 1, 1);

	NRF24_WriteBit(radio, 28, 0, 1);

	NRF24_FlushRX(radio);
	HAL_Delay(50);
	NRF24_FlushTX(radio);

	NRF24_ClearInterrupts(radio);
}

uint32_t send_delay_timer;

void NRF24_PacketSend(NRF24_RADIO *radio, data_packet *packet)
{
	static uint8_t txBuf[2];
	static uint8_t rxBuf[2];

	waiting_for_ack = 1;

	NRF24_FlushTX(radio);

	txBuf[0] = 0b10100000;
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(radio->spiHandle, txBuf, rxBuf, 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(radio->spiHandle, packet->payload, radio->garbageRxBuf, packet->width, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
	NRF24_WriteBit(radio, 0, 0, 0);//Go into TX mode
	send_delay_timer = GetMicros();
	while(GetMicrosDifference(&send_delay_timer) < 20);
	//HAL_Delay(2);
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

void NRF24_PacketRead(NRF24_RADIO *radio)
{
	static uint8_t txBuf[32];
	static uint8_t rxBuf[32];
	static uint8_t packet_width;

	txBuf[0] = 0b01100000;
	txBuf[1] = 0;

	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(radio->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
	packet_width = rxBuf[1];

	for(int i = 0; i < 32; i++)
	{
		txBuf[i] = 0;
		rxBuf[i] = 0;
	}

	txBuf[0] = 0b01100001;
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(radio->spiHandle, txBuf, rxBuf, (uint8_t)(packet_width + 1), HAL_MAX_DELAY);

	//HAL_SPI_TransmitReceive(radio->spiHandle, txBuf, rxBuf, packet_width, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

	if(rxBuf[1] != 0x00)
	{
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		packets_to_receive[packets_to_receive_counter].width = packet_width;
		packets_to_receive[packets_to_receive_counter].reliable = 1;

		for(int i = 0; i < packet_width; i++)
		{
			packets_to_receive[packets_to_receive_counter].payload[i] = rxBuf[i + 1];
		}

		if(packets_to_receive_counter < 30)
			packets_to_receive_counter++;
	}
}

/*
 * Change single bit in register
 */
void NRF24_WriteBit(NRF24_RADIO *radio, uint8_t address, uint8_t bit_add, uint8_t val)
{
	static uint8_t txBuf[2];
	static uint8_t rxBuf[2];

	txBuf[0] = address + 32;
    txBuf[1] = NRF24_GetAddress(radio, address);

	if(val == 1)
	{
		txBuf[1] |= 1 << bit_add;
	}
	else
	{
		txBuf[1] &= ~(1 << bit_add);
	}

	HAL_GPIO_WritePin(radio->csnPinPort, radio->csnPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(radio->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(radio->csnPinPort, radio->csnPin, GPIO_PIN_SET);
}

/*
 * Get NRF24 radio register value
 */
uint8_t NRF24_GetAddress(NRF24_RADIO *radio, uint8_t address)
{
	static uint8_t txBuf[2];
	txBuf[0] = address;
	txBuf[1] = 0x00;

	static uint8_t reg_value[2];

	HAL_GPIO_WritePin(radio->csnPinPort, radio->csnPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(radio->spiHandle, txBuf, reg_value, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(radio->csnPinPort, radio->csnPin, GPIO_PIN_SET);

	return reg_value[1];
}

void NRF24_FlushTX(NRF24_RADIO *radio)
{
	static uint8_t txBuf[1];
	static uint8_t rxBuf[1];

	txBuf[0] = 0b11100001;//Flush TX
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(radio->spiHandle, txBuf, rxBuf, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

void NRF24_FlushRX(NRF24_RADIO *radio)
{
	static uint8_t txBuf[1];
	static uint8_t rxBuf[1];

	txBuf[0] = 0b11100010;//Flush RX
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(radio->spiHandle, txBuf, rxBuf, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

void NRF24_ClearInterrupts(NRF24_RADIO *radio)
{
	if((NRF24_GetAddress(radio, 7) >> 4) & 0x01)//RT interrupt
	{
		NRF24_WriteBit(radio, 7, 4, 1);
	}

	if((NRF24_GetAddress(radio, 7) >> 5) & 0x01)//TX interrupt
	{
		NRF24_WriteBit(radio, 7, 5, 1);
	}

	if((NRF24_GetAddress(radio, 7) >> 6) & 0x01)//RX interrupt
	{
		NRF24_WriteBit(radio, 7, 6, 1);
	}
}
