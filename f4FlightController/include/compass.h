#ifndef _COMPASS_H
#define _COMPASS_H

#include "Arduino.h"

void SetupCompass(void);
void ReadCompass(void);
void CalibrateCompass(void);
void CalculateCompassCalibration(void);
void CalculateHeadingDifference(float ang1, float ang2);

extern uint8_t COMPASS_ADDRESS;

extern uint32_t calibrate_compass_timer;

extern float compass_heading;
extern float heading_difference_return;

extern int16_t compass_x_min, compass_x_max;
extern int16_t compass_y_min, compass_y_max;
extern int16_t compass_z_min, compass_z_max;

#endif