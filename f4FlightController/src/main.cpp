#include "Arduino.h"
#include "Wire.h"
#include "imu.h"
#include "string.h"
#include "tim_IO.h"
#include "wiretelem.h"

uint8_t IMU_ADDRESS = 0x68;

void SetupAutomaticPacketI2C(void);

int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
float gyro_x_val, gyro_y_val, gyro_z_val;
int16_t imu_temp;
int16_t raw_acc_x, raw_acc_y, raw_acc_z;

uint64_t telem_timer;
uint64_t loop_timer;

int32_t frequencyRead, frequencyRead1 = 1500, frequencyRead2 = 1500, frequencyRead3 = 995, frequencyRead4 = 1500, frequencyRead5 = 995, frequencyRead6 = 995, input_freq = 0, throttle = 995;
int32_t esc1Channel, esc2Channel, esc3Channel, esc4Channel;
int32_t esc1_output, esc2_output, esc3_output, esc4_output;

void setup()
{
  Timers_Setup();
  delay(500);
  //SerialUSB.begin();

  Wire.setClock((uint32_t)400000);
  Wire.begin();

  telem_wire_setup();
  delay(250);

  SetupIMU();
  delay(250);

  SetupAutomaticPacketI2C();
  delay(250);

  pinMode(PC1, OUTPUT);
  pinMode(PC2, OUTPUT);

  loop_timer = micros();
  delay(8000);
}

void loop()
{
  if (micros() - loop_timer > 4000) //Main loop
  {
    loop_timer = micros();
    ReadIMU();
  }

  telem_loop();
}

void SetupAutomaticPacketI2C()
{
  packet_buf[0].id = GYRO_PACKET;
  PopulatePacketBuf(packet_buf[0].payload, &raw_gyro_x, 0);
  PopulatePacketBuf(packet_buf[0].payload, &raw_gyro_y, 2);
  PopulatePacketBuf(packet_buf[0].payload, &raw_gyro_z, 4);
  packet_buf[0].width = 7;  //3 * sizeof(int16_t) + 1(byte for id)
  packet_count = 1;
}