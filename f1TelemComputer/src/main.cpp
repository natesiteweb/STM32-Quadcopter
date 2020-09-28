#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "nrf24radio.h"
#include "telemdata.h"

#define MY_ADDRESS 0x04

uint8_t CE_pin = PB12;
uint8_t CSN_pin = PA8;
uint8_t IRQ_pin = PA11;
uint8_t MOSI_pin = PB15;
uint8_t MISO_pin = PB14;
uint8_t SCK_pin = PB13;

void I2C_Receive(int howMany);

byte send_data[33];
uint8_t received_data[33];

HardwareSerial Serial3(PB11, PB10);

uint8_t received_num;
volatile uint8_t wire_buf[40];
volatile bool i2c_receive_flag = false;

uint8_t waiting_for_ack = 0;

volatile uint8_t radio_irq_flag;
uint8_t radio_receive_flag;
uint8_t radio_transmit_flag;
uint8_t radio_retransmit_flag;

uint8_t packet_count = 0;

/////////-TELEMETRY DATA-/////////

int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;

//////////////////////////////////

void setup()
{
  pinMode(PA6, OUTPUT);
  pinMode(PA7, OUTPUT);
  pinMode(IRQ_pin, INPUT_PULLUP);

  Serial.begin(57600);

  delay(5000);
  NRF_Init();

  packet_buf[0].id = GYRO_PACKET;
  PopulatePacketBuf(packet_buf[0].payload, &gyro_x, 0);
  PopulatePacketBuf(packet_buf[0].payload, &gyro_y, 2);
  PopulatePacketBuf(packet_buf[0].payload, &gyro_z, 4);
  packet_buf[0].width = 7; //3 * sizeof(int16_t) + 1(byte for id)
  packet_count = 1;

  //Serial3.begin(9600);

  delay(100);

  pps_timer = millis();

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin(MY_ADDRESS);

  Wire.onReceive(I2C_Receive);
}

void loop()
{
  radio_loop();

  if (millis() - pps_timer >= 1000)
  {
    pps_timer = millis();

    Serial.print("Sent: ");
    Serial.print(sent_packet_counter);
    Serial.print(" ACKs: ");
    Serial.print(ack_packet_counter);
    Serial.print(" X: ");
    Serial.print(((float)gyro_x) / 65.5);
    Serial.print(" Y: ");
    Serial.print(((float)gyro_y) / 65.5);
    Serial.print(" Z: ");
    Serial.println(((float)gyro_z) / 65.5);

    sent_packet_counter = 0;
    ack_packet_counter = 0;
  }

  if (i2c_receive_flag)
  {
    i2c_receive_flag = false;

    switch (wire_buf[0])
    {
    case GYRO_PACKET:
      gyro_x = (((wire_buf[1] & 0xFF) << 8) | (wire_buf[2] & 0xFF));
      gyro_y = (((wire_buf[3] & 0xFF) << 8) | (wire_buf[4] & 0xFF));
      gyro_z = (((wire_buf[5] & 0xFF) << 8) | (wire_buf[6] & 0xFF));
      break;
    }
  }
}

void I2C_Receive(int howMany)
{
  Wire.readBytes((uint8_t *)wire_buf, howMany);

  i2c_receive_flag = true;
}