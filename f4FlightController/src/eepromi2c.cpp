#include "Arduino.h"
#include "Wire.h"
#include "eepromi2c.h"

float_union float_u;
int16_union int16_u;

int address_index = 0;
int page_index = 0;

uint8_t started_page_write = 0;

uint8_t eeprom_buf[32];

void EEPROM_Save_Page(uint16_t address)
{
    Wire.beginTransmission(EEPROM_ADDRESS);
    Wire.write((uint8_t)((address >> 8) & 0xFF));
    Wire.write((uint8_t)(address & 0xFF));
    Wire.write(eeprom_buf, (uint8_t)32);
    Wire.endTransmission();
}

void EEPROM_Load_Page(uint16_t address)
{
    Wire.beginTransmission(EEPROM_ADDRESS);
    Wire.write((uint8_t)((address >> 8) & 0xFF));
    Wire.write((uint8_t)(address & 0xFF));
    Wire.endTransmission();

    Wire.requestFrom(EEPROM_ADDRESS, (uint8_t)32);

    Wire.readBytes((uint8_t *)eeprom_buf, (uint8_t)32);
}

void EEPROM_Float_Write(uint8_t buf_index, float num)
{
    float_u.num = num;

    eeprom_buf[buf_index] = float_u.data[0];
    eeprom_buf[buf_index + 1] = float_u.data[1];
    eeprom_buf[buf_index + 2] = float_u.data[2];
    eeprom_buf[buf_index + 3] = float_u.data[3];
}

float EEPROM_Float_Read(uint8_t buf_index)
{
    float_u.data[0] = eeprom_buf[buf_index];
    float_u.data[1] = eeprom_buf[buf_index + 1];
    float_u.data[2] = eeprom_buf[buf_index + 2];
    float_u.data[3] = eeprom_buf[buf_index + 3];

    return float_u.num;
}

void EEPROM_Int16_Write(uint8_t buf_index, int16_t num)
{
    int16_u.num = num;

    eeprom_buf[buf_index] = int16_u.data[0];
    eeprom_buf[buf_index + 1] = int16_u.data[1];
}

int16_t EEPROM_Int16_Read(uint8_t buf_index)
{
    int16_u.data[0] = eeprom_buf[buf_index];
    int16_u.data[1] = eeprom_buf[buf_index + 1];

    return int16_u.num;
}

void EEPROM_Single_Byte_Write(uint16_t address, uint8_t num)
{
    Wire.beginTransmission(EEPROM_ADDRESS);
    Wire.write((uint8_t)((address >> 8) & 0xFF));
    Wire.write((uint8_t)(address & 0xFF));
    Wire.write(num);
    Wire.endTransmission();
}

uint8_t EEPROM_Single_Byte_Read(uint16_t address)
{
    Wire.beginTransmission(EEPROM_ADDRESS);
    Wire.write((uint8_t)((address >> 8) & 0xFF));
    Wire.write((uint8_t)(address & 0xFF));
    Wire.endTransmission();

    Wire.requestFrom(EEPROM_ADDRESS, (uint8_t)1);
    uint8_t val = Wire.read();
    return val;
}

void EEPROM_Clear_Buf()
{
    for(int i = 0; i < 32; i++)
    {
        eeprom_buf[i] = 0x00;
    }
}