#include "Arduino.h"
#include "Wire.h"
#include "imu.h"
#include "string.h"
#include "tim_IO.h"
#include "wiretelem.h"
#include "eepromi2c.h"
#include "gyro_pid.h"
#include "bmp280.h"

uint8_t IMU_ADDRESS = 0x68;
uint8_t BMP_ADDRESS = 0x76;
uint8_t EEPROM_ADDRESS = 0x50;

void SetupAutomaticPacketI2C(void);
void ParseControllerInput(void);

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
 **/
uint8_t flight_mode = 1;

float acc_x_offset = 0;
float acc_y_offset = 0;

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

  kp_pitch = kp_roll;
  ki_pitch = ki_roll;
  kd_pitch = kd_roll;

  gyro_pid_timer = micros();
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
    yaw_angle = total_gyro_z_angle;

    if (pid_altitude_setpoint < bmp_setpoint - 0.05 && flight_mode == 3)
      pid_altitude_setpoint += 0.00175;
    else if (pid_altitude_setpoint > bmp_setpoint + 0.05 && flight_mode == 3)
      pid_altitude_setpoint -= 0.00175;

    if ((bmp_reading_index + 1) % 5 == 0)
    {
      Calculate_Baro_Altitude();

      if (frequencyRead6 > 1600)
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
    }

    ParseControllerInput();

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
  auto_packet_buf[1].width = 6;

  auto_packet_count = 2;
}

void ParseControllerInput()
{
  if(frequencyRead6 < 1300)
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
    if (flight_mode != 3)
    {
      flight_mode = 3;
      captured_throttle = frequencyRead3;
      altitude_pid_timer = micros() - 20000;
    }

    throttle = pid_altitude_output + captured_throttle;
  }
}