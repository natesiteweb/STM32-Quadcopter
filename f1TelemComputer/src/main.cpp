#include "Arduino.h"
#include "Wire.h"

#define MY_ADDRESS 0x04

void I2C_Receive(int howMany);

HardwareSerial Serial3(PB11, PB10);

uint8_t received_num;
volatile char wire_buf[40];
volatile bool i2c_receive_flag = false;

void setup() 
{
  pinMode(PA6, OUTPUT);
  pinMode(PA7, OUTPUT);

  Serial.begin(115200);
  Serial3.begin(9600);

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin(MY_ADDRESS);

  Wire.onReceive(I2C_Receive);
}

void loop() 
{
  if(i2c_receive_flag)
  {
    i2c_receive_flag = false;
    Serial.print("Received: ");
    Serial.println((char*)wire_buf);
  }
}

void I2C_Receive(int howMany)
{
  Wire.readBytes((char*)wire_buf, howMany);

  i2c_receive_flag = true;
}