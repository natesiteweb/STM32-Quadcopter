#include "Arduino.h"
#include "Wire.h"
#include "imu.h"
#include "string.h"

#define IMU_ADDRESS 0x68

int16_t raw_gyro_x = 0;
float gyro_x_val = 0;

uint64_t imu_timer, blink_timer;
uint8_t blink_state = 0x00;
uint64_t loop_timer;

TwoWire Wire2(PB11, PB10);

char buf_to_send[40] = "Hello";

void setup()
{
  delay(500);
  SerialUSB.begin();

  Wire.begin();
  Wire.setClock((uint32_t)400000);

  Wire2.setClock((uint32_t)400000);
  Wire2.begin();

  SetupIMU(IMU_ADDRESS);

  pinMode(PC1, OUTPUT);
  pinMode(PC2, OUTPUT);

  imu_timer = micros();
  loop_timer = micros();
  blink_timer = millis();
  delay(10000);

  Wire.beginTransmission(0x50);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x3E);
  Wire.endTransmission();

  delay(1000);

  SerialUSB.print("Accessing address 0: ");
  uint8_t read_byte;

  Wire.beginTransmission(0x50);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)0x50, (uint8_t)1);
  read_byte = Wire.read();

  SerialUSB.println((uint8_t)read_byte, HEX);
}

void loop()
{
  if (micros() - imu_timer > 20000)
  {
    imu_timer = micros();
    ReadIMU(IMU_ADDRESS);
    //SerialUSB.println(gyro_x_val);
  }

  if (millis() - blink_timer > 250)
  {
    //snprintf((char*)buf_to_send, sizeof(buf_to_send), "%d.%d", (int)gyro_x_val, (int)((gyro_x_val - (int)gyro_x_val) * 100));

    Wire2.beginTransmission(0x04);
    Wire2.write((char *)buf_to_send);
    Wire2.endTransmission();

    blink_timer = millis();
    digitalWrite(PC1, blink_state);
    digitalWrite(PC2, 0x01 - blink_state);

    blink_state = ~blink_state & 0x01;
  }
}