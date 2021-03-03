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

extern float kp_roll, kp_pitch, kp_yaw;
extern float ki_roll, ki_pitch, ki_yaw;
extern float kd_roll, kd_pitch, kd_yaw;

#endif /* INC_CONTROL_LOGIC_H_ */
