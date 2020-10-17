#ifndef _PID_LOGIC_H
#define _PID_LOGIC_H

#include "Arduino.h"

void GyroPID(void);
void AltitudePID(void);
void GPSPID(void);
void ResetGPSVariables(void);

extern uint8_t flight_mode;

extern uint32_t gyro_pid_timer;

extern float kp_roll, kp_pitch, kp_yaw;
extern float ki_roll, ki_pitch, ki_yaw;
extern float kd_roll, kd_pitch, kd_yaw;
extern float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
extern float pid_roll_i, pid_pitch_i, pid_yaw_i, pid_altitude_i;
extern float pid_roll_last_error, pid_pitch_last_error, pid_yaw_last_error, pid_altitude_last_error;

extern float kp_altitude;
extern float ki_altitude;
extern float kd_altitude;

extern float pid_altitude_setpoint;

extern int32_t throttle;
extern int32_t captured_throttle;

extern uint32_t altitude_pid_timer;

extern int32_t pid_roll_output, pid_pitch_output, pid_yaw_output;
extern int32_t pid_altitude_output;

extern int32_t esc1_output, esc2_output, esc3_output, esc4_output;

extern float kp_gps;
extern float ki_gps;
extern float kd_gps;

extern float pid_gps_roll_i, pid_gps_pitch_i;
extern float calculated_lat_error, calculated_lon_error;
extern float last_calculated_lat_error, last_calculated_lon_error;
extern float lat_add, lon_add;

extern uint32_t gps_timer;
extern float gps_timer_modifier;

extern int32_t gps_roll_modifier, gps_pitch_modifier;

extern float latitude_error_over_time_total, longitude_error_over_time_total;
extern float latitude_error_over_time[40], longitude_error_over_time[40];
extern uint8_t gps_error_over_time_reading_index;

extern int32_t latitude_table[40];
extern int32_t longitude_table[40];

extern int32_t lat_setpoint;
extern int32_t lon_setpoint;

extern int32_t raw_latitude;
extern int32_t raw_longitude;
extern int32_t last_raw_latitude;
extern int32_t last_raw_longitude;
extern uint8_t new_gps_data;

extern uint8_t sat_count;

#endif