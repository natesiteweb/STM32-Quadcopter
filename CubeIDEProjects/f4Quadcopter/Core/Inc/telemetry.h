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
void ClearManualBuffer(void);
void ClearPrintBuffer(void);
void PrintManualPacket(void);
void AddToAutoBuffer(uint8_t buf_index, uint8_t *num, uint8_t size);
void AddIDToManualBuffer(uint8_t id);
void AddToManualBuffer(uint8_t *num, uint8_t size);
void ReadReceiveBuffer(uint8_t *output, uint8_t size);

typedef struct
{
	uint8_t id;
	uint8_t *payload[32];
	uint8_t width[32];
	uint8_t reliable;
	uint8_t var_count;
	uint8_t total_width;
	uint8_t send_rate;
	uint8_t cycles_since_last_sent;
} data_packet_pointer;

typedef struct
{
	uint8_t payload[35];
	uint8_t width;
	uint8_t reliable;
} data_packet;

extern char print_text_buffer[32];

extern volatile uint8_t telem_send_buffer[35];
extern volatile uint8_t telem_receive_buffer[35];
extern data_packet empty_data_packet;
extern volatile uint8_t ack_rate_counter;
extern uint8_t ack_rate;
extern volatile uint8_t waiting_to_rx;
extern volatile uint8_t rx_done;
extern volatile uint8_t tx_done;
extern data_packet manual_packet_buffer[32];
extern uint8_t manual_packet_count;

extern data_packet_pointer auto_packet_buffer[32];
extern uint8_t auto_packet_count;

extern uint32_t acks_per_second_timer;
extern uint32_t acks_counted;
extern uint32_t acks_per_second;

enum
{
    GYRO_PACKET = 0x01,
	PID_OUTPUT_PACKET = 0x02,
    PID_GAIN_FIRST_PACKET = 0x03,
    ALTITUDE_PACKET = 0x06,
    ALTITUDE_SET_PACKET = 0x07,
    PID_GAIN_SECOND_PACKET = 0x04,
    GPS_PACKET = 0x08,
    PRINT_PACKET = 0x09,
    STATUS_FIRST_PACKET = 0x0A,

    PID_GAIN_FIRST_REQUEST = 0xF3,
    PID_GAIN_SECOND_REQUEST = 0xF4,
    PID_GAIN_FIRST_UPDATE_REQUEST = 0xF5,
    PID_GAIN_SECOND_UPDATE_REQUEST = 0xF6,
	CALIBRATE_GYRO_REQUEST = 0xF7,
    ALTITUDE_REQUEST = 0xF8,
    ALTITUDE_SET_REQUEST = 0xF9,
    CALIBRATE_COMPASS_REQUEST = 0xFA,
    CALIBRATE_ESC_REQUEST = 0xFB,
    GPS_PACKET_UPDATE_REQUEST = 0xFD,
    GPS_PACKET_REQUEST = 0xFE,
    GPS_HOLD_COPY_BUFFER_REQUEST = 0xE0,
    FLIGHT_MODE_UPDATE_REQUEST = 0xFC,
};

#endif /* INC_TELEMETRY_H_ */
