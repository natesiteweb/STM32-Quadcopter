#ifndef _TELEMDATA_H
#define _TELEMDATA_H

#include "Arduino.h"

void PopulatePacketBuf(uint8_t **buf, float *num, int start_index);
void PopulatePacketBuf(uint8_t **buf, uint32_t *num, int start_index);
void PopulatePacketBuf(uint8_t **buf, uint16_t *num, int start_index);
void PopulatePacketBuf(uint8_t **buf, int16_t *num, int start_index);

extern uint8_t packet_count;

uint8_t *Float_To_Bytes(float num);
uint8_t *UInt32_To_Bytes(uint32_t num);
uint8_t *UInt16_To_Bytes(uint16_t num);
uint8_t *Int32_To_Bytes(int32_t num);
uint8_t *Int16_To_Bytes(int16_t num);

enum
{
    GYRO_PACKET = 0x01
};

typedef union
{
    float num;
    uint8_t data[4];
} float_union;

typedef union
{
    uint32_t num;
    uint8_t data[4];
} uint32_union;

typedef union
{
    uint16_t num;
    uint8_t data[2];
} uint16_union;

typedef union
{
    int32_t num;
    uint8_t data[4];
} int32_union;

typedef union
{
    int16_t num;
    uint8_t data[2];
} int16_union;

typedef struct
{
    uint8_t id;
    uint8_t *payload[31];
    uint8_t width;
} data_packet_pointer;

#endif