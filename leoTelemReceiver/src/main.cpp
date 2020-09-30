#include "Arduino.h"
#include "SPI.h"
#include "nrf24radio.h"

#define MY_ADDRESS 0x04

uint8_t CE_pin = 7;
uint8_t CSN_pin = A5;
uint8_t IRQ_pin = 3;

void get_data(void);

//SPIClass rfspi(MOSI_pin, MISO_pin, SCK_pin);
uint8_t received_data[32];
uint8_t received_data_len = 0;

uint8_t ack_payload_width = 1;
uint8_t ack_payload_from_serial[32];

payload_buf ack_payload_buf[32]; //FIFO
uint8_t ack_payload_buf_counter = 0;

//HardwareSerial Serial3(PB11, PB10);

uint8_t received_num;
volatile char wire_buf[40];
volatile bool i2c_receive_flag = false;

void setup()
{
  ack_payload_buf[0].payload[0] = 0x1C;
  ack_payload_buf[0].payload[1] = 0x20;
  ack_payload_buf[0].width = 2;
  //pinMode(PA6, OUTPUT);
  pinMode(5, OUTPUT);

  Serial.begin(115200);

  delay(5000);
  NRF_Init();

  delay(100);

  digitalWrite(5, HIGH);

  //Wire.setSDA(PB7);
  //Wire.setSCL(PB6);
  //Wire.begin(MY_ADDRESS);

  //Wire.onReceive(I2C_Receive);
}

byte testState = 0;

uint8_t dummy_read;

void loop()
{
  if (radio_irq_flag == 1)
  {
    radio_irq_flag = 0;
    get_data();

    for (int i = 0; i < received_data_len; i++)
    {
      Serial.write(received_data[i]);
    }

    NRF_Write_Bit(7, 6, 1); //Clear RX Interrupt
  }
  else
  {
    if (Serial.available() > 0)
    {
      if (ack_payload_buf_counter < 30)
      {
        ack_payload_width = 0;
        ack_payload_buf_counter++;

        while (Serial.available() > 0)
        {
          ack_payload_from_serial[ack_payload_width] = Serial.read();
          ack_payload_buf[ack_payload_buf_counter].payload[ack_payload_width] = ack_payload_from_serial[ack_payload_width];

          ack_payload_width++;
        }

        ack_payload_buf[ack_payload_buf_counter].width = ack_payload_width;
      }
      else
      {
        while (Serial.available() > 0)
        {
          dummy_read = Serial.read();
        }
      }
    }
  }
}