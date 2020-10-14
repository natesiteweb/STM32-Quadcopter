#include "Arduino.h"
#include "Wire.h"
#include "imu.h"

int16_t raw_gyro_x = 0, raw_gyro_y = 0, raw_gyro_z = 0;
float gyro_x_val, gyro_y_val, gyro_z_val;
float roll_angle, pitch_angle, yaw_angle;
float gyro_x_calibration, gyro_y_calibration, gyro_z_calibration;
float total_gyro_x_angle, total_gyro_y_angle, total_gyro_z_angle;
float acc_x_calibration, acc_y_calibration, acc_z_calibration;
int16_t imu_temp;
int16_t raw_acc_x, raw_acc_y, raw_acc_z;
float acc_x_val, acc_y_val, acc_z_val;
float acc_magnitude;

void SetupIMU()
{
  Wire.beginTransmission(IMU_ADDRESS); //Wake
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDRESS); //Gyro 500dps scale
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDRESS); //Low pass filter
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.beginTransmission(IMU_ADDRESS); //Acceleromter +/- 8g scale
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
}

void ReadIMU(bool calibrating)
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

  if (!calibrating)
  {
    raw_gyro_x -= gyro_x_calibration;
    raw_gyro_y -= gyro_y_calibration;
    raw_gyro_z -= gyro_z_calibration;
  }
}

void CalibrateIMU()
{
  roll_angle = 0;
  pitch_angle = 0;
  yaw_angle = 0;

  gyro_x_calibration = 0;
  gyro_y_calibration = 0;
  gyro_z_calibration = 0;

  total_gyro_x_angle = 0;
  total_gyro_y_angle = 0;
  total_gyro_z_angle = 0;

  gyro_x_val = 0;
  gyro_y_val = 0;
  gyro_z_val = 0;

  for (int i = 0; i < 2000; i++)
  {
    ReadIMU(true);

    gyro_x_calibration += raw_gyro_x;
    gyro_y_calibration += raw_gyro_y;
    gyro_z_calibration += raw_gyro_z;

    delay(3);
  }

  gyro_x_calibration /= 2000;
  gyro_y_calibration /= 2000;
  gyro_z_calibration /= 2000;
}