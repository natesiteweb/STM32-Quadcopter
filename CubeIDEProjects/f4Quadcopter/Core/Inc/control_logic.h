/*
 * control_logic.h
 *
 *  Created on: Mar 2, 2021
 *      Author: Nate
 */

#ifndef INC_CONTROL_LOGIC_H_
#define INC_CONTROL_LOGIC_H_

#include "stm32f4xx_hal.h"

void Calculate_Attitude(void);
void Motor_PID(void);
void Calculate_Motor_Outputs(void);
void Calculate_Altitude_PID(void);
void Control_Loop(void);
void Parse_Requested_State(int32_t requested_state);
void Launch_Behavior(void);
void Land_Behavior(void);

extern int32_t pid_roll_output, pid_pitch_output, pid_yaw_output;
extern float kp_roll, kp_yaw;
extern float ki_roll, ki_yaw;
extern float kd_roll, kd_yaw;

extern int32_t esc1_output, esc2_output, esc3_output, esc4_output;

extern float calculated_bmp_altitude;

extern uint8_t program_buffer[512];

extern uint8_t launched, launching, landing;
extern uint8_t ready_for_next_command;
extern uint8_t manual_mode;

extern float hover_throttle;
extern uint32_t launch_timer;

enum
{
	LANDED = 0x00,
	LAUNCHED = 0x01
};

#endif /* INC_CONTROL_LOGIC_H_ */
