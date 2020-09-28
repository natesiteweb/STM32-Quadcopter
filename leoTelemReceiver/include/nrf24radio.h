#ifndef _NRF24RADIO_H
#define _NRF24RADIO_H

void NRF_Init(void);
void _RECEIVE(void);
void NRF_Flush_TX(void);
void NRF_Flush_RX(void);
void NRF_Clear_Interrupts(void);
void NRF_Get_Packet_Width(void);
void NRF_Set_Ack_Payload(void);
void NRF_Get_Address(byte address);
void NRF_Write_Bit(byte address, byte bit_add, byte val);
void NRF_Write_Byte(byte address, byte val);

extern uint8_t CE_pin;
extern uint8_t CSN_pin;
extern uint8_t IRQ_pin;

extern uint8_t received_data[32];
extern uint8_t received_data_len;
extern uint8_t payload_width;

extern volatile uint8_t radio_irq_flag;

typedef struct
{
  uint8_t payload[32];
  uint8_t width;
} payload_buf;

typedef union
{
    float num;
    uint8_t data[4];
} float_union;

extern payload_buf ack_payload_buf[32];
extern uint8_t ack_payload_buf_counter;

//volatile
#endif