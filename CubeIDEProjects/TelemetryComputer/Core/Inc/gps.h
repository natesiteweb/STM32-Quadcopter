/*
 * gps.h
 *
 *  Created on: Apr 24, 2021
 *      Author: Nate
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "stm32f1xx_hal.h"

void GPS_Read(void);
void GPS_Send_Packet(void);
void GPS_Init(void);
void Custom_GPS_Send(uint8_t *data, uint8_t size);

extern uint8_t gps_buffer[99];
extern volatile uint8_t gps_buffer_index;
extern volatile uint8_t new_gps_line;

extern uint8_t waiting_for_gps_packet_sent;

#endif /* INC_GPS_H_ */
