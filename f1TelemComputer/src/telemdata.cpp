#include "Arduino.h"
#include "telemdata.h"

float_union f_union;
uint32_union ui32_union;
uint16_union ui16_union;
int32_union i32_union;
int16_union i16_union;

uint8_t *Float_To_Bytes(float num)
{
    f_union.num = num;
    return f_union.data;
}

uint8_t *UInt32_To_Bytes(uint32_t num)
{
    ui32_union.num = num;
    return ui32_union.data;
}

uint8_t *UInt16_To_Bytes(uint16_t num)
{
    ui16_union.num = num;
    return ui16_union.data;
}

uint8_t *Int32_To_Bytes(int32_t num)
{
    i32_union.num = num;
    return i32_union.data;
}

uint8_t *Int16_To_Bytes(int16_t num)
{
    i16_union.num = num;
    return i16_union.data;
}

void PopulatePacketBuf(uint8_t **buf, float *num, int start_index)
{
    buf[start_index] = (uint8_t *)num;
    buf[start_index + 1] = ((uint8_t *)num) + 1;
    buf[start_index + 2] = ((uint8_t *)num) + 2;
    buf[start_index + 3] = ((uint8_t *)num) + 3;
}

void PopulatePacketBuf(uint8_t **buf, uint32_t *num, int start_index)
{
    buf[start_index] = (uint8_t *)num;
    buf[start_index + 1] = ((uint8_t *)num) + 1;
    buf[start_index + 2] = ((uint8_t *)num) + 2;
    buf[start_index + 3] = ((uint8_t *)num) + 3;
}

void PopulatePacketBuf(uint8_t **buf, uint16_t *num, int start_index)
{
    buf[start_index] = (uint8_t *)num;
    buf[start_index + 1] = ((uint8_t *)num) + 1;
}

void PopulatePacketBuf(uint8_t **buf, int16_t *num, int start_index)
{
    buf[start_index] = (uint8_t *)num;
    buf[start_index + 1] = ((uint8_t *)num) + 1;
}