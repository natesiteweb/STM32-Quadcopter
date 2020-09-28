#ifndef _NRF24RADIO_H
#define _NRF24RADIO_H

#include "Arduino.h"
#include "telemdata.h"

void NRF_Init(void);
void _RF24_IRQ(void);
void radio_loop(void);
void NRF_Clear_Interrupts(void);
void NRF_Send_Packet(uint8_t *buf[32], uint8_t id, uint8_t buf_width);
void NRF_Get_Address(byte address);
void NRF_Write_Bit(byte address, byte bit_add, byte val);

extern uint8_t CE_pin;
extern uint8_t CSN_pin;
extern uint8_t IRQ_pin;
extern uint8_t MOSI_pin;
extern uint8_t MISO_pin;
extern uint8_t SCK_pin;

extern uint8_t waiting_for_ack;
extern int sent_packet_counter;
extern int ack_packet_counter;
extern uint8_t ack_width;
extern uint8_t ack_payload_test;
extern uint64_t pps_timer;

extern volatile uint8_t radio_irq_flag;
extern uint8_t radio_receive_flag;
extern uint8_t radio_transmit_flag;
extern uint8_t radio_retransmit_flag;

extern uint8_t received_data[33];
extern data_packet_pointer packet_buf[32];

#endif