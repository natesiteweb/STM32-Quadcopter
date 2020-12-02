#include "Arduino.h"
#include "Wire.h"
#include "imu.h"
#include "tim_IO.h"
#include "wiretelem.h"
#include "pid_logic.h"
#include "bmp280.h"
#include "compass.h"

float kp_roll, kp_pitch, kp_yaw;
float ki_roll, ki_pitch, ki_yaw;
float kd_roll, kd_pitch, kd_yaw;
float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
float pid_roll_i, pid_pitch_i, pid_yaw_i;
float pid_roll_last_error = 0, pid_pitch_last_error = 0, pid_yaw_last_error = 0, pid_altitude_last_error = 0;

float pid_error_temp;

int32_t pid_roll_output, pid_pitch_output, pid_yaw_output;

int32_t esc1_output, esc2_output, esc3_output, esc4_output;

uint32_t gyro_pid_timer, altitude_pid_timer;
float time_since_last_gyro_pid, time_since_last_altitude_pid;

float kp_altitude, kp_altitude_actual;
float ki_altitude;
float kd_altitude;
float pid_altitude_i;
float pid_altitude_setpoint = 2;

int32_t pid_altitude_output;

float pid_altitude_over_time[10], pid_altitude_over_time_total = 0;
uint8_t pid_altitude_over_time_index = 0;

int32_t last_frequency6 = 1000;
int32_t captured_throttle;

float kp_gps, kp_gps_actual;
float ki_gps;
float kd_gps, kd_gps_actual;
float kd2_gps;

float pid_gps_roll_i = 0, pid_gps_pitch_i = 0;
float calculated_lat_error = 0, calculated_lon_error = 0;
float last_calculated_lat_error = 0, last_calculated_lon_error = 0;
float lat_add = 0, lon_add = 0;

uint32_t gps_timer;
float gps_timer_modifier;

int32_t gps_roll_modifier = 0, gps_pitch_modifier = 0;

float latitude_error_over_time_total, longitude_error_over_time_total;
float latitude_error_over_time[40], longitude_error_over_time[40];
uint8_t gps_error_over_time_reading_index = 0;

/**
 * 0: Most recent GPS position
 * 1: Home setpoint
 * 2: Realtime setpoint(Setpoint but interpolated)
 * 3: Setpoint(not interpolated)
 * 4: Last setpoint
 * 5: Hold position buffer
 * 6+: Waypoints
**/
int32_t latitude_table[40];
int32_t longitude_table[40];

uint8_t waypoint_count = 0; //Count NOT index

int32_t lat_setpoint = 1;
int32_t lon_setpoint = 1;

int32_t raw_latitude = 1;
int32_t raw_longitude = 1;
int32_t last_raw_latitude = 1;
int32_t last_raw_longitude = 1;
uint8_t new_gps_data = 0;

uint8_t sat_count = 0;

float altitude_table[40];
uint8_t auto_pilot_command_table[80];

uint32_t delay_command_timer;
uint8_t is_auto_pilot = 0;

int32_t gps_roll_modifier_north;
int32_t gps_pitch_modifier_north;

void GyroPID()
{
    time_since_last_gyro_pid = (float)(micros() - gyro_pid_timer) / (float)4000;
    gyro_pid_timer = micros();

    if (frequencyRead1 > 1508)
        pid_roll_setpoint = (frequencyRead1 - 1508);
    else if (frequencyRead1 < 1492)
        pid_roll_setpoint = (frequencyRead1 - 1492);
    else
        pid_roll_setpoint = 0;

    if (frequencyRead2 > 1508)
        pid_pitch_setpoint = (frequencyRead2 - 1508);
    else if (frequencyRead2 < 1492)
        pid_pitch_setpoint = (frequencyRead2 - 1492);
    else
        pid_pitch_setpoint = 0;

    if (frequencyRead4 > 1508)
        pid_yaw_setpoint = (frequencyRead4 - 1508);
    else if (frequencyRead4 < 1492)
        pid_yaw_setpoint = (frequencyRead4 - 1492);
    else
        pid_yaw_setpoint = 0;

    if(flight_mode >= 3)
    {
        throttle = pid_altitude_output + captured_throttle;
    }

    if (flight_mode >= 4 && flight_mode != 5) //GPS Hold
    {
        pid_roll_setpoint += gps_roll_modifier;
        pid_pitch_setpoint += gps_pitch_modifier;
    }

    pid_roll_setpoint -= (roll_angle * 10);
    pid_pitch_setpoint -= (pitch_angle * 10);

    pid_roll_setpoint /= 3.0;
    pid_pitch_setpoint /= 3.0;
    pid_yaw_setpoint /= 3.0;

    //PID Roll
    pid_error_temp = pid_roll_setpoint - gyro_x_val;
    pid_roll_i += (pid_error_temp * ki_roll * time_since_last_gyro_pid);

    if (pid_roll_i > 300)
        pid_roll_i = 300;
    else if (pid_roll_i < -300)
        pid_roll_i = -300;

    pid_roll_output = ((pid_error_temp * kp_roll * time_since_last_gyro_pid) + pid_roll_i + ((pid_error_temp - pid_roll_last_error) * time_since_last_gyro_pid));

    pid_roll_last_error = pid_error_temp;

    if (pid_roll_output > 300)
        pid_roll_output = 300;
    else if (pid_roll_output < -300)
        pid_roll_output = -300;

    //PID Pitch
    pid_error_temp = pid_pitch_setpoint - gyro_y_val;
    pid_pitch_i += (pid_error_temp * ki_pitch * time_since_last_gyro_pid);

    if (pid_pitch_i > 300)
        pid_pitch_i = 300;
    else if (pid_pitch_i < -300)
        pid_pitch_i = -300;

    pid_pitch_output = ((pid_error_temp * kp_pitch * time_since_last_gyro_pid) + pid_pitch_i + ((pid_error_temp - pid_pitch_last_error) * time_since_last_gyro_pid));

    pid_pitch_last_error = pid_error_temp;

    if (pid_pitch_output > 300)
        pid_pitch_output = 300;
    else if (pid_pitch_output < -300)
        pid_pitch_output = -300;

    //PID Yaw
    pid_error_temp = pid_yaw_setpoint - gyro_z_val;
    pid_yaw_i += (pid_error_temp * ki_yaw * time_since_last_gyro_pid);

    if (pid_yaw_i > 300)
        pid_yaw_i = 300;
    else if (pid_yaw_i < -300)
        pid_yaw_i = -300;

    pid_yaw_output = ((pid_error_temp * kp_yaw * time_since_last_gyro_pid) + pid_yaw_i + ((pid_error_temp - pid_yaw_last_error) * time_since_last_gyro_pid));

    pid_yaw_last_error = pid_error_temp;

    if (pid_yaw_output > 300)
        pid_yaw_output = 300;
    else if (pid_yaw_output < -300)
        pid_yaw_output = -300;

    if (flight_mode > 1 && frequencyRead3 > 1010)
    {
        digitalWrite(PC1, HIGH);

        esc1_output = throttle + pid_roll_output + pid_pitch_output - pid_yaw_output;
        esc2_output = throttle - pid_roll_output + pid_pitch_output + pid_yaw_output;
        esc3_output = throttle - pid_roll_output - pid_pitch_output - pid_yaw_output;
        esc4_output = throttle + pid_roll_output - pid_pitch_output + pid_yaw_output;
    }
    else
    {
        digitalWrite(PC1, LOW);

        pid_error_temp = 0;
        pid_roll_i = 0;
        pid_pitch_i = 0;
        pid_yaw_i = 0;
        pid_roll_setpoint = 0;
        pid_pitch_setpoint = 0;
        pid_yaw_setpoint = 0;
        pid_roll_output = 0;
        pid_pitch_output = 0;
        pid_yaw_output = 0;

        esc1_output = 1000;
        esc2_output = 1000;
        esc3_output = 1000;
        esc4_output = 1000;
    }

    if (esc1_output > 1995)
        esc1_output = 2000;
    else if (esc1_output < 1020)
        esc1_output = 1000;

    if (esc2_output > 1995)
        esc2_output = 2000;
    else if (esc2_output < 1020)
        esc2_output = 1000;

    if (esc3_output > 1995)
        esc3_output = 2000;
    else if (esc3_output < 1020)
        esc3_output = 1000;

    if (esc4_output > 1995)
        esc4_output = 2000;
    else if (esc4_output < 1020)
        esc4_output = 1000;

    ESC1Timer->setCaptureCompare(4, esc1_output, MICROSEC_COMPARE_FORMAT);
    ESC1Timer->setCaptureCompare(3, esc2_output, MICROSEC_COMPARE_FORMAT);
    ESC1Timer->setCaptureCompare(2, esc3_output, MICROSEC_COMPARE_FORMAT);
    ESC1Timer->setCaptureCompare(1, esc4_output, MICROSEC_COMPARE_FORMAT);

    /*ESC2Timer->setCaptureCompare(4, frequencyRead3, MICROSEC_COMPARE_FORMAT);
    ESC2Timer->setCaptureCompare(3, frequencyRead3, MICROSEC_COMPARE_FORMAT);
    ESC2Timer->setCaptureCompare(2, frequencyRead3, MICROSEC_COMPARE_FORMAT);
    ESC2Timer->setCaptureCompare(1, frequencyRead3, MICROSEC_COMPARE_FORMAT);*/
}

void AltitudePID()
{
    time_since_last_altitude_pid = (float)(micros() - altitude_pid_timer) / (float)20000;
    altitude_pid_timer = micros();

    pid_error_temp = pid_altitude_setpoint - bmp_altitude;
    pid_altitude_i += (pid_error_temp * ki_altitude * time_since_last_altitude_pid);

    if (pid_error_temp > 1.60 || pid_error_temp < -1.60)
    {
        kp_altitude_actual = kp_altitude * 2.5;
    }
    else
        kp_altitude_actual = kp_altitude;

    pid_altitude_over_time_total -= pid_altitude_over_time[pid_altitude_over_time_index];
    pid_altitude_over_time[pid_altitude_over_time_index] = ((pid_error_temp - pid_altitude_last_error) * kd_altitude * time_since_last_altitude_pid);
    pid_altitude_over_time_total += pid_altitude_over_time[pid_altitude_over_time_index];

    pid_altitude_over_time_index++;

    if (pid_altitude_over_time_index == 5)
        pid_altitude_over_time_index = 0;

    if (pid_altitude_i > 600)
        pid_altitude_i = 600;
    else if (pid_altitude_i < -600)
        pid_altitude_i = -600;

    pid_altitude_output = ((pid_error_temp * kp_altitude * time_since_last_altitude_pid) + pid_altitude_i + pid_altitude_over_time_total);

    pid_altitude_last_error = pid_error_temp;
}

uint8_t gps_counter = 0;

void GPSPID()
{
    if ((micros() - gps_timer >= 20000 && gps_counter < 9) || new_gps_data == 1)
    {
        gps_timer_modifier = (float)((micros() - gps_timer) / (float)20000);
        gps_timer = micros();

        if (new_gps_data == 0)
        {
            gps_counter++;

            calculated_lat_error += (lat_add/* + lat_modifier_add*/);// * gps_timer_modifier;
            calculated_lon_error += (lon_add/* + lon_modifier_add*/);// * gps_timer_modifier;
        }
        else
        {
            gps_counter = 0;
        }

        latitude_error_over_time_total -= latitude_error_over_time[gps_error_over_time_reading_index];
        latitude_error_over_time[gps_error_over_time_reading_index] = (calculated_lat_error - last_calculated_lat_error);// * gps_timer_modifier;
        latitude_error_over_time_total += latitude_error_over_time[gps_error_over_time_reading_index];

        longitude_error_over_time_total -= longitude_error_over_time[gps_error_over_time_reading_index];
        longitude_error_over_time[gps_error_over_time_reading_index] = (calculated_lon_error - last_calculated_lon_error);// * gps_timer_modifier;
        longitude_error_over_time_total += longitude_error_over_time[gps_error_over_time_reading_index];

        last_calculated_lat_error = calculated_lat_error;
        last_calculated_lon_error = calculated_lon_error;

        gps_error_over_time_reading_index++;

        if (gps_error_over_time_reading_index == 30)
            gps_error_over_time_reading_index = 0;

        pid_error_temp = calculated_lon_error;

        gps_roll_modifier_north = ((pid_error_temp * kp_gps_actual/* * gps_timer_modifier*/) + (longitude_error_over_time_total * kd_gps_actual));

        pid_error_temp = calculated_lat_error;

        gps_pitch_modifier_north = ((pid_error_temp * kp_gps_actual/* * gps_timer_modifier*/) + (latitude_error_over_time_total * kd_gps_actual));

        gps_roll_modifier = (int32_t)(((float)gps_roll_modifier_north * cos(yaw_angle * 0.017453)) + ((float)gps_pitch_modifier_north * sin(yaw_angle * 0.017453)));
        gps_pitch_modifier = (int32_t)(((float)gps_pitch_modifier_north * cos(yaw_angle * 0.017453)) + ((float)gps_roll_modifier_north * sin(yaw_angle * -0.017453)));

        if (gps_roll_modifier > 400)
            gps_roll_modifier = 400;
        else if (gps_roll_modifier < -400)
            gps_roll_modifier = -400;

        if (gps_pitch_modifier > 400)
            gps_pitch_modifier = 400;
        else if (gps_pitch_modifier < -400)
            gps_pitch_modifier = -400;

        new_gps_data = 0;
    }
}

void ResetGPSVariables()
{
    gps_error_over_time_reading_index = 0;

    for (int i = 0; i < 40; i++)
    {
        latitude_error_over_time[i] = 0;
        longitude_error_over_time[i] = 0;
    }

    latitude_error_over_time_total = 0;
    longitude_error_over_time_total = 0;

    last_calculated_lat_error = 0;
    last_calculated_lon_error = 0;

    gps_roll_modifier_north = 0;
    gps_pitch_modifier_north = 0;

    gps_roll_modifier = 0;
    gps_pitch_modifier = 0;

    kp_gps_actual = kp_gps;
    kd_gps_actual = kd_gps;

    lat_add = 0;
    lon_add = 0;

    gps_timer = micros() - 20000;
}