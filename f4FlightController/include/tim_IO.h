#ifndef _TIM_IO_H
#define _TIM_IO_H

#include "Arduino.h"

void Rollover_IT_callback(void);
void TIMINPUT_Capture_Falling_IT_callback(void);
void TIMINPUT_Capture_Rising_IT_callback(void);
void Update_IT_callback(void);
void Compare_IT_callback(void);
void Timers_Setup(void);

extern HardwareTimer *ESC1Timer;

extern int32_t frequencyRead, frequencyRead1, frequencyRead2, frequencyRead3, frequencyRead4, frequencyRead5, frequencyRead6, input_freq;

#endif