/*
 * control_logic.h
 *
 *  Created on: Mar 2, 2021
 *      Author: Nate
 */

#ifndef INC_CONTROL_LOGIC_H_
#define INC_CONTROL_LOGIC_H_

#include "stm32f4xx_hal.h"

void Motor_PID(void);
void Calculate_Motor_Outputs(void);
void Calculate_Altitude_PID(void);
void Control_Loop(void);

extern int32_t pid_roll_output, pid_pitch_output, pid_yaw_output;
extern float kp_roll, kp_yaw;
extern float ki_roll, ki_yaw;
extern float kd_roll, kd_yaw;

extern int32_t esc1_output, esc2_output, esc3_output, esc4_output;

extern float calculated_bmp_altitude;

extern uint8_t program_buffer[512];

#endif /* INC_CONTROL_LOGIC_H_ */
