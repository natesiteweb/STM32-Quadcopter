/*
 * control_loop.h
 *
 *  Created on: Apr 25, 2021
 *      Author: Nate
 */

#ifndef INC_CONTROL_LOOP_H_
#define INC_CONTROL_LOOP_H_

#include "stm32f4xx_hal.h"

void Control_Loop(void);
uint16_t Parse_Command(uint8_t *cmd_array, uint16_t cmd_index, uint8_t high_priority);
void Parse_Requested_State(int32_t requested_state);
void TryToLaunchDependency(uint8_t high_priority);

typedef struct
{
	uint8_t *var;
	uint8_t width;
	uint8_t var_index;
	uint8_t protected;
} direct_access_var;

extern direct_access_var direct_access_variables[256];
extern uint8_t program_buffer[512];

extern uint8_t high_priority_program_buffer[32];
extern uint8_t high_priority_program_counter;
extern uint8_t high_priority_program_width;

extern uint8_t ready_for_next_command, ready_for_next_command_high_priority;

#endif /* INC_CONTROL_LOOP_H_ */
