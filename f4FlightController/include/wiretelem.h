#ifndef _WIRETELEM_H
#define _WIRETELEM_H

#include "Arduino.h"
#include "Wire.h"

void telem_loop(void);
void telem_wire_setup(void);

void SendGPSPacket(uint8_t gps_index);

void PopulatePacketBuf(uint8_t **buf, float *num, int start_index);
void PopulatePacketBuf(uint8_t **buf, uint32_t *num, int start_index);
void PopulatePacketBuf(uint8_t **buf, int32_t *num, int start_index);
void PopulatePacketBuf(uint8_t **buf, uint16_t *num, int start_index);
void PopulatePacketBuf(uint8_t **buf, int16_t *num, int start_index);
void PopulatePacketBuf(uint8_t **buf, uint8_t *num, int start_index);

void PopulateManualPacket(float num);
void PopulateManualPacket(int32_t num);
void PopulateManualPacket(uint8_t num);
float ReadManualPacket_Float();
int32_t ReadManualPacket_Int32();
uint32_t ReadManualPacket_UInt32();

void TelemPrintDebug(char txt_to_print[30], uint8_t message_length);

extern uint8_t telem_rate_counter;
extern uint8_t telem_read_counter;

extern uint32_t telem_timer;

typedef struct
{
    uint8_t id;
    uint8_t *payload[31];
    uint8_t width;
} data_packet_pointer;

typedef struct
{
    uint8_t id;
    uint8_t payload[31];
    uint8_t width;
} data_packet;

extern data_packet_pointer auto_packet_buf[32];
extern uint8_t auto_packet_count;
extern data_packet packet_buf[32];

enum
{
    GYRO_PACKET = 0x01,
    PID_GAIN_FIRST_PACKET = 0x03,
    ALTITUDE_PACKET = 0x06,
    ALTITUDE_SET_PACKET = 0x07,
    PID_GAIN_SECOND_PACKET = 0x04,
    GPS_PACKET = 0x08,
    PRINT_PACKET = 0x09,

    PID_GAIN_FIRST_REQUEST = 0xF3,
    PID_GAIN_SECOND_REQUEST = 0xF4,
    PID_GAIN_FIRST_UPDATE_REQUEST = 0xF5,
    PID_GAIN_SECOND_UPDATE_REQUEST = 0xF6,
    ALTITUDE_REQUEST = 0xF8,
    ALTITUDE_SET_REQUEST = 0xF9,
    CALIBRATE_COMPASS_REQUEST = 0xFA,
    CALIBRATE_ESC_REQUEST = 0xFB,
    GPS_PACKET_UPDATE_REQUEST = 0xFD,
    GPS_PACKET_REQUEST = 0xFE,
    GPS_HOLD_COPY_BUFFER_REQUEST = 0xE0
};

typedef union 
{
    int32_t num;
    uint8_t data[4];
} int32_union;

typedef union 
{
    uint32_t num;
    uint8_t data[4];
} uint32_union;

#endif