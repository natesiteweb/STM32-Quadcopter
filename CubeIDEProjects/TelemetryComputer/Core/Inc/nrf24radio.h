/*
 * nrf24radio.h
 *
 *  Created on: Dec 3, 2020
 *      Author: Nate
 */

#ifndef INC_NRF24RADIO_H_
#define INC_NRF24RADIO_H_

#include "stm32f1xx_hal.h"

typedef struct
{

	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef *csnPinPort;
	GPIO_TypeDef *cePinPort;
	uint16_t csnPin;
	uint16_t cePin;

	uint8_t packetRxBuf[32];

	uint8_t garbageRxBuf[32];

} NRF24_RADIO;

typedef struct
{
	uint8_t payload[35];//32 is packet width, 33 is 1 for reliable, 34 is request bit(1 if master is asking for data)
	uint8_t width;
	uint8_t reliable;
} data_packet;

extern volatile uint8_t radio_irq_flag;
extern volatile uint8_t waiting_for_ack;

extern data_packet reliable_packets_to_gcs[32];
extern volatile data_packet current_i2c_packet;
extern data_packet unreliable_packet;
extern uint8_t reliable_packet_to_gcs_counter;

extern volatile data_packet packets_to_receive[32];
extern uint8_t packets_to_receive_counter;


void NRF24_Init(NRF24_RADIO *radio);

uint8_t NRF24_GetAddress(NRF24_RADIO *radio, uint8_t address);
void NRF24_WriteBit(NRF24_RADIO *radio, uint8_t address, uint8_t bit_add, uint8_t val);
void NRF24_ClearInterrupts(NRF24_RADIO *radio);

void NRF24_PacketSend(NRF24_RADIO *radio, data_packet *packet);
void NRF24_PacketRead(NRF24_RADIO *radio);
void NRF24_FlushTX(NRF24_RADIO *radio);
void NRF24_FlushRX(NRF24_RADIO *radio);

#endif /* INC_NRF24RADIO_H_ */
