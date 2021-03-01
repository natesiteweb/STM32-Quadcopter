/*
 * telemetry.h
 *
 *  Created on: Feb 28, 2021
 *      Author: Nate
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include "stm32f4xx_hal.h"

void telem_loop(void);
void AddToAutoBuffer(uint8_t buf_index, uint8_t *num, uint8_t size);

typedef struct
{
	uint8_t id;
	uint8_t *payload[32];
	uint8_t width[32];
	uint8_t var_count;
	uint8_t total_width;
} data_packet_pointer;

typedef struct
{
	uint8_t payload[35];
	uint8_t width;
} data_packet;

extern volatile uint8_t telem_send_buffer[35];
extern data_packet empty_data_packet;
extern uint8_t telem_receive_buffer[35];
extern uint8_t new_telem_received;
extern uint32_t telem_receive_timout_timer;
extern volatile uint8_t ack_rate_counter;
extern uint8_t ack_rate;
extern uint8_t waiting_for_ack;

extern data_packet manual_packet_buffer[32];
extern uint8_t manual_packet_count;

extern data_packet_pointer auto_packet_buffer[32];
extern uint8_t auto_packet_count;

#endif /* INC_TELEMETRY_H_ */
