#ifndef _GPS_H
#define _GPS_H

#include "Arduino.h"

void Setup_GPS(void);
void Read_GPS(void);

void ReadBatteryVoltage(void);

extern HardwareSerial Serial2;

extern int32_t latitude, longitude;
extern uint8_t sat_count;
extern bool gps_fix;
extern uint8_t new_gps_data;

extern uint8_t is_calibrating;
extern uint32_t is_calibrating_timer;

extern uint32_t battery_voltage;

#endif