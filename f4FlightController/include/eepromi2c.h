#ifndef _EEPROMI2C_H
#define _EEPROMI2C_H

extern uint8_t EEPROM_ADDRESS;

void EEPROM_Clear_Buf(void);
void EEPROM_Float_Write(uint8_t buf_index, float num);
float EEPROM_Float_Read(uint8_t buf_index);

void EEPROM_Save_Page(uint16_t address);
void EEPROM_Load_Page(uint16_t address);

uint8_t EEPROM_Single_Byte_Read(uint16_t address);
void EEPROM_Single_Byte_Write(uint16_t address, uint8_t num);

extern uint8_t eeprom_buf[32];

typedef union
{
    float num;
    uint8_t data[4];
} float_union;

extern float_union float_u;


#endif