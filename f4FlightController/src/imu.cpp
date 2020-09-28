#include "Arduino.h"
#include "Wire.h"
#include "imu.h"

void SetupIMU()
{
  Wire.beginTransmission(IMU_ADDRESS);  //Wake
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDRESS);  //Gyro 500dps scale
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDRESS);  //Low pass filter
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDRESS);  //Acceleromter +/- 8g scale
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
}

void ReadIMU()
{
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(IMU_ADDRESS, (uint8_t)14);

  raw_acc_x = Wire.read() << 8 | Wire.read();
  raw_acc_y = Wire.read() << 8 | Wire.read();
  raw_acc_z = Wire.read() << 8 | Wire.read();
  imu_temp = Wire.read() << 8 | Wire.read();
  raw_gyro_x = Wire.read() << 8 | Wire.read();
  raw_gyro_y = Wire.read() << 8 | Wire.read();
  raw_gyro_z = Wire.read() << 8 | Wire.read();

  gyro_x_val = (float)raw_gyro_x / 65.5;
  gyro_y_val = (float)raw_gyro_y / 65.5;
  gyro_z_val = (float)raw_gyro_z / 65.5;
}