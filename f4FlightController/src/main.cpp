#include "Arduino.h"
#include "Wire.h"
#include "imu.h"
#include "string.h"
#include "tim_IO.h"
#include "wiretelem.h"
#include "eepromi2c.h"
#include "pid_logic.h"
#include "bmp280.h"
#include "compass.h"

uint8_t IMU_ADDRESS = 0x68;
uint8_t BMP_ADDRESS = 0x76;
uint8_t EEPROM_ADDRESS = 0x50;
uint8_t COMPASS_ADDRESS = 0x1E;

void SetupAutomaticPacketI2C(void);
void ParseControllerInput(void);
void InterpolateGPSCoords(void);

uint32_t telem_timer;
uint32_t loop_timer;
float time_since_last_main_loop;

int32_t frequencyRead, frequencyRead1 = 1500, frequencyRead2 = 1500, frequencyRead3 = 995, frequencyRead4 = 1500, frequencyRead5 = 2000, frequencyRead6 = 995, input_freq = 0, throttle = 995;
int32_t esc1Channel, esc2Channel, esc3Channel, esc4Channel;

float angle_calculation_bias = 0.9985;

/**
 * 1: Disarmed
 * 2: Manual; autolevel
 * 3: Manual; autolevel; altitude hold
 * 4: Manual; autolevel; altitude hold; GPS hold
 **/
uint8_t flight_mode = 1;

float battery_voltage = 0;

uint8_t gps_interpolate_counter = 0;

float acc_x_offset = 0;
float acc_y_offset = -2.6;

void setup()
{
  Wire.setClock((uint32_t)400000);
  Wire.begin();
  delay(250);

  Timers_Setup();
  delay(500);
  //SerialUSB.begin();

  telem_wire_setup();
  delay(250);

  SetupIMU();

  delay(250);

  readCoefficients();
  setSampling();

  delay(250);

  SetupAutomaticPacketI2C();
  delay(250);

  pinMode(PC1, OUTPUT);
  pinMode(PC2, OUTPUT);

  loop_timer = micros();
  delay(8000);

  //EEPROM_Float_Write(0x00, 0x00, (float)4);

  EEPROM_Clear_Buf();
  EEPROM_Load_Page(0);

  kp_roll = EEPROM_Float_Read(0);
  ki_roll = EEPROM_Float_Read((uint16_t)4);
  kd_roll = EEPROM_Float_Read((uint16_t)8);

  kp_yaw = EEPROM_Float_Read((uint16_t)12);
  ki_yaw = EEPROM_Float_Read((uint16_t)16);
  kd_yaw = EEPROM_Float_Read((uint16_t)20);

  EEPROM_Clear_Buf();
  EEPROM_Load_Page(32);
  kp_altitude = EEPROM_Float_Read((uint16_t)0);
  ki_altitude = EEPROM_Float_Read((uint16_t)4);
  kd_altitude = EEPROM_Float_Read((uint16_t)8);

  kp_gps = EEPROM_Float_Read((uint16_t)12);
  ki_gps = EEPROM_Float_Read((uint16_t)16);
  kd_gps = EEPROM_Float_Read((uint16_t)20);

  EEPROM_Clear_Buf();
  EEPROM_Load_Page(64);
  compass_x_min = EEPROM_Int16_Read((uint16_t)0);
  compass_x_max = EEPROM_Int16_Read((uint16_t)2);
  compass_y_min = EEPROM_Int16_Read((uint16_t)4);
  compass_y_max = EEPROM_Int16_Read((uint16_t)6);
  compass_z_min = EEPROM_Int16_Read((uint16_t)8);
  compass_z_max = EEPROM_Int16_Read((uint16_t)10);

  kp_pitch = kp_roll;
  ki_pitch = ki_roll;
  kd_pitch = kd_roll;

  kp_gps_actual = kp_gps;

  delay(250);

  SetupCompass();

  delay(250);

  gyro_pid_timer = micros();

  analogReadResolution(12);
}

void loop()
{
  if (micros() - loop_timer >= 4000) //Main loop
  {
    time_since_last_main_loop = micros() - loop_timer;
    loop_timer = micros();

    ReadIMU(false);

    gyro_x_val = (float)raw_gyro_x / 65.5;
    gyro_y_val = (float)raw_gyro_y / -65.5;
    gyro_z_val = (float)raw_gyro_z / -65.5;

    acc_magnitude = sqrt(((float)raw_acc_x * (float)raw_acc_x) + ((float)raw_acc_y * (float)raw_acc_y) + ((float)raw_acc_z * (float)raw_acc_z));

    if (abs(raw_acc_y) < acc_magnitude)
    {
      if (acc_magnitude != 0)
      {
        acc_x_val = asin((float)raw_acc_y / acc_magnitude) * 57.296;
      }
    }

    if (abs(raw_acc_x) < acc_magnitude)
    {
      if (acc_magnitude != 0)
      {
        acc_y_val = asin((float)raw_acc_x / acc_magnitude) * 57.296;
      }
    }

    total_gyro_x_angle += (gyro_x_val) * (time_since_last_main_loop / 1000000);
    total_gyro_y_angle += (gyro_y_val) * (time_since_last_main_loop / 1000000);
    total_gyro_z_angle += (gyro_z_val) * (time_since_last_main_loop / 1000000);

    total_gyro_x_angle += (total_gyro_y_angle * sin(gyro_z_val * 0.01745 * (time_since_last_main_loop / 1000000)));
    total_gyro_y_angle -= (total_gyro_x_angle * sin(gyro_z_val * 0.01745 * (time_since_last_main_loop / 1000000)));

    total_gyro_x_angle = (total_gyro_x_angle * angle_calculation_bias) + ((acc_x_val + acc_x_offset) * (1.0000 - angle_calculation_bias));
    total_gyro_y_angle = (total_gyro_y_angle * angle_calculation_bias) + ((acc_y_val + acc_y_offset) * (1.0000 - angle_calculation_bias));

    roll_angle = total_gyro_x_angle;
    pitch_angle = total_gyro_y_angle;

    if (total_gyro_z_angle < 0)
      total_gyro_z_angle += 360;
    else if (total_gyro_z_angle >= 360)
      total_gyro_z_angle -= 360;

    CalculateHeadingDifference(total_gyro_z_angle, compass_heading);

    if (heading_difference_return > 10 || heading_difference_return < -10)
      total_gyro_z_angle = compass_heading;

    yaw_angle = total_gyro_z_angle;

    if (pid_altitude_setpoint < bmp_setpoint - 0.05 && flight_mode >= 3)
      pid_altitude_setpoint += 0.00175;
    else if (pid_altitude_setpoint > bmp_setpoint + 0.05 && flight_mode >= 3)
      pid_altitude_setpoint -= 0.00175;

    if (flight_mode >= 4)
    {
      if (gps_interpolate_counter == 20)
      {
        gps_interpolate_counter = 0;

        InterpolateGPSCoords();
      }

      gps_interpolate_counter++;
    }

    ParseControllerInput();

    if ((bmp_reading_index + 1) % 5 == 0)
    {
      Calculate_Baro_Altitude();

      if (flight_mode > 2)
        AltitudePID();
      else
      {
        pid_altitude_output = 0;
        pid_altitude_i = 0;
        pid_altitude_last_error = 0;
      }
    }

    bmp_reading_index++;

    if (bmp_reading_index == 20)
    {
      bmp_reading_index = 0;

      ReadCompass();
    }

    if (flight_mode >= 4)
    {
      GPSPID();
    }

    GyroPID();

    telem_loop();
  }
}

void SetupAutomaticPacketI2C()
{
  auto_packet_buf[0].id = GYRO_PACKET;
  PopulatePacketBuf(auto_packet_buf[0].payload, &raw_gyro_x, 0);
  PopulatePacketBuf(auto_packet_buf[0].payload, &raw_gyro_y, 2);
  PopulatePacketBuf(auto_packet_buf[0].payload, &raw_gyro_z, 4);
  PopulatePacketBuf(auto_packet_buf[0].payload, &roll_angle, 6);
  PopulatePacketBuf(auto_packet_buf[0].payload, &pitch_angle, 10);
  PopulatePacketBuf(auto_packet_buf[0].payload, &yaw_angle, 14);
  PopulatePacketBuf(auto_packet_buf[0].payload, &time_since_last_main_loop, 18);
  PopulatePacketBuf(auto_packet_buf[0].payload, &frequencyRead3, 22);
  auto_packet_buf[0].width = 27; //variables plus id

  auto_packet_buf[1].id = ALTITUDE_PACKET;
  PopulatePacketBuf(auto_packet_buf[1].payload, &bmp_altitude, 0);
  PopulatePacketBuf(auto_packet_buf[1].payload, &flight_mode, 4);
  PopulatePacketBuf(auto_packet_buf[1].payload, &gps_roll_modifier, 5);
  PopulatePacketBuf(auto_packet_buf[1].payload, &gps_pitch_modifier, 9);
  PopulatePacketBuf(auto_packet_buf[1].payload, &compass_heading, 13);
  PopulatePacketBuf(auto_packet_buf[1].payload, &battery_voltage, 17);
  auto_packet_buf[1].width = 22;

  auto_packet_count = 2;
}

void ParseControllerInput()
{
  if (frequencyRead6 < 1300)
  {
    flight_mode = 1;
  }
  if (frequencyRead6 > 1300 && frequencyRead6 < 1800)
  {
    flight_mode = 2;
    throttle = frequencyRead3;

    captured_throttle = 0;
  }
  else if (frequencyRead6 > 1750)
  {
    if (flight_mode < 4) //GPS hold
    {
      flight_mode = 4;
      captured_throttle = frequencyRead3;
      ResetTimers();

      latitude_table[1] = latitude_table[0];
      longitude_table[1] = longitude_table[0];

      latitude_table[2] = latitude_table[1];
      longitude_table[2] = longitude_table[1];

      latitude_table[3] = latitude_table[1];
      longitude_table[3] = longitude_table[1];

      latitude_table[4] = latitude_table[1];
      longitude_table[4] = longitude_table[1];

      last_lat_interp = latitude_table[1];
      last_lon_interp = longitude_table[1];

      lat_modifier_add = 0;
      lon_modifier_add = 0;

      lat_modifier = 0;
      lon_modifier = 0;

      SendGPSPacket((uint8_t)1); //Send home position
    }
    /*else if(sat_count < 3)
    {
      flight_mode = 2;
    }*/
  }
}

void ResetTimers()
{
  loop_timer = micros() - 4000;
  altitude_pid_timer = micros() - 20000;
  gyro_pid_timer = micros() - 4000;

  telem_rate_counter = 0;
  telem_read_counter = 0;

  ResetGPSVariables();
}

int32_t lat_interp_difference = 0;
int32_t lon_interp_difference = 0;
float lat_modifier = 0;
float lon_modifier = 0;

float lat_modifier_add = 0;
float lon_modifier_add = 0;

int32_t last_lat_interp = 0;
int32_t last_lon_interp = 0;

void InterpolateGPSCoords()
{
  lat_interp_difference = latitude_table[3] - latitude_table[4];
  lon_interp_difference = longitude_table[3] - longitude_table[4];

  kp_gps_actual = kp_gps;

  if (abs(latitude_table[2] - latitude_table[3]) < 2 || abs(longitude_table[2] - longitude_table[3]) < 2)
  {
    latitude_table[2] = latitude_table[3];
    longitude_table[2] = longitude_table[3];

    latitude_table[4] = latitude_table[3];
    longitude_table[4] = longitude_table[3];

    if (lat_modifier != 0 && lon_modifier != 0)
    {
      //SendGPSPacket((uint8_t)2);
    }

    lat_modifier_add = 0;
    lon_modifier_add = 0;

    lat_modifier = 0;
    lon_modifier = 0;
  }
  else
  {
    kp_gps_actual = kp_gps * 0.75;

    last_lat_interp = latitude_table[2];
    last_lon_interp = longitude_table[2];

    if (abs(lat_interp_difference) > abs(lon_interp_difference))
    {
      if (latitude_table[2] < latitude_table[3])
        lat_modifier += 1;
      else
        lat_modifier -= 1;

      lon_modifier = (((float)lon_interp_difference / (float)lat_interp_difference) * lat_modifier);

      latitude_table[2] = latitude_table[4] + lat_modifier;
      longitude_table[2] = longitude_table[4] + lon_modifier;
    }
    else if (abs(lat_interp_difference) < abs(lon_interp_difference))
    {
      if (longitude_table[2] < longitude_table[3])
        lon_modifier += 1;
      else
        lon_modifier -= 1;

      lat_modifier = (((float)lat_interp_difference / (float)lon_interp_difference) * lon_modifier);

      latitude_table[2] = latitude_table[4] + lat_modifier;
      longitude_table[2] = longitude_table[4] + lon_modifier;
    }
    else if (abs(lat_interp_difference) == abs(lon_interp_difference))
    {
      if (latitude_table[2] < latitude_table[3])
        lat_modifier += 1;
      else
        lat_modifier -= 1;

      lon_modifier = (((float)lon_interp_difference / (float)lat_interp_difference) * lat_modifier);

      latitude_table[2] = latitude_table[4] + lat_modifier;
      longitude_table[2] = longitude_table[4] + lon_modifier;
    }

    lat_modifier_add = (float)(last_lat_interp - latitude_table[2]) / (float)4.2;
    lon_modifier_add = (float)(longitude_table[2] - last_lon_interp) / (float)4.2;

    //SendGPSPacket((uint8_t)2);
  }
}