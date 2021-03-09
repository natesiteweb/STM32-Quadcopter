/*
 * eeprom.h
 *
 *  Created on: Mar 2, 2021
 *      Author: Nate
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stm32f4xx_hal.h"

void EEPROM_Clear_Buffer(void);
void EEPROM_Save_Page(uint16_t address);
void EEPROM_Read_Page(uint16_t address, uint8_t size);
void EEPROM_Write_Buffer(uint8_t *num, uint8_t size);
void EEPROM_Read_Buffer(uint8_t *output, uint8_t size);


volatile uint8_t eeprom_read_happening;
uint8_t eeprom_write_buffer_width;
uint8_t eeprom_read_write_buffer[34];
uint8_t eeprom_read_buffer_index;

#endif /* INC_EEPROM_H_ */
