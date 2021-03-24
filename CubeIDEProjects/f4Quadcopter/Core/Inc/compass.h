/*
 * compass.h
 *
 *  Created on: Mar 23, 2021
 *      Author: Nate
 */

#ifndef INC_COMPASS_H_
#define INC_COMPASS_H_

#include "stm32f4xx_hal.h"

void Setup_Compass(void);
void Read_Compass(void);
void Calibrate_Compass(void);
void Calculate_Compass_Calibration(void);
void CalculateHeadingDifference(float ang1, float ang2);

extern float compass_heading;

extern int16_t compass_x_min, compass_x_max;
extern int16_t compass_y_min, compass_y_max;
extern int16_t compass_z_min, compass_z_max;
extern float heading_difference_return;

#endif /* INC_COMPASS_H_ */
