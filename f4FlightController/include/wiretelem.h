#ifndef _WIRETELEM_H
#define _WIRETELEM_H

#include "Arduino.h"
#include "Wire.h"

void telem_loop(void);
void telem_wire_setup(void);

void PopulatePacketBuf(uint8_t **buf, float *num, int start_index);
void PopulatePacketBuf(uint8_t **buf, uint32_t *num, int start_index);
void PopulatePacketBuf(uint8_t **buf, uint16_t *num, int start_index);
void PopulatePacketBuf(uint8_t **buf, int16_t *num, int start_index);



extern uint64_t telem_timer;

typedef struct
{
    uint8_t id;
    uint8_t *payload[31];
    uint8_t width;
} data_packet_pointer;

extern data_packet_pointer packet_buf[32];
extern uint8_t packet_count;

enum
{
    GYRO_PACKET = 0x01
};

#endif