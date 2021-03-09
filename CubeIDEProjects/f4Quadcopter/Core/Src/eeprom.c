/*
 * eeprom.c
 *
 *  Created on: Mar 2, 2021
 *      Author: Nate
 */


#include "main.h"
#include "i2c.h"
#include "eeprom.h"

static const uint8_t EEPROM_ADDR = 0x50 << 1;

volatile uint8_t eeprom_read_happening = 0;

uint8_t eeprom_write_buffer_width = 2;
uint8_t eeprom_read_write_buffer[34];
uint8_t eeprom_read_buffer_index = 0;

void EEPROM_Clear_Buffer()
{
	for(int i = 0; i < 34; i++)
	{
		eeprom_read_write_buffer[i] = 0x00;
	}
}

//This also works with single bytes
void EEPROM_Save_Page(uint16_t address)
{
	eeprom_read_write_buffer[0] = (uint8_t)((address >> 8) & 0xFF);
	eeprom_read_write_buffer[1] = (uint8_t)(address & 0xFF);
	HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR, (uint8_t *)eeprom_read_write_buffer, eeprom_write_buffer_width, 50);
}

//This also works with single bytes(set size to 1)
void EEPROM_Read_Page(uint16_t address, uint8_t size)
{
	HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDR, address, I2C_MEMADD_SIZE_16BIT, (uint8_t *)eeprom_read_write_buffer, size, 50);
}

void EEPROM_Write_Buffer(uint8_t *num, uint8_t size)
{
	for(int i = 0; i < size; i++)
	{
		eeprom_read_write_buffer[eeprom_write_buffer_width + i] = *((uint8_t *)num + i);
	}

	eeprom_write_buffer_width += size;
}

void EEPROM_Read_Buffer(uint8_t *output, uint8_t size)
{
	for(int i = 0; i < size; i++)
	{
		*(((uint8_t *)output) + i) = eeprom_read_write_buffer[eeprom_read_buffer_index + i];
	}

	eeprom_read_buffer_index += size;
}
