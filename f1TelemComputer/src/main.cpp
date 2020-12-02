#include "Arduino.h"
//#include "Wire.h"
//#include "SPI.h"
//#include "nrf24radio.h"
//#include "gps.h"
#include "telemdata.h"

#define MY_ADDRESS 0x04

uint8_t CE_pin = PB12;
uint8_t CSN_pin = PA8;
uint8_t IRQ_pin = PA11;
uint8_t MOSI_pin = PB15;
uint8_t MISO_pin = PB14;
uint8_t SCK_pin = PB13;

//void I2C_Receive(int howMany);
//void I2C_Request(void);

byte send_data[35];
uint8_t received_data[35];

//HardwareSerial Serial3(PB11, PB10);

uint8_t received_num;
volatile uint8_t wire_buf[45];
volatile bool i2c_receive_flag = false;

uint8_t waiting_for_ack = 0;

volatile uint8_t radio_irq_flag;
volatile uint8_t radio_receive_flag = 0;
volatile uint8_t radio_transmit_flag;
volatile uint8_t radio_retransmit_flag;

uint8_t auto_packet_count = 0;

uint8_t is_calibrating = 0;
uint32_t is_calibrating_timer;

uint32_t battery_read_timer;

/////////-TELEMETRY DATA-/////////

int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;
uint32_t battery_voltage = 0;

//////////////////////////////////

uint32_t debug_i2c_timer, debug_loop_timer;

int sent_packet_counter = 0;
int ack_packet_counter = 0;


/**
* Packets to be updated automatically
* 0: Gyro 
*/
data_packet_pointer auto_packet_buf[36];

data_packet packet_buf[36];     //Packet buffer FIFO from F4 master
uint8_t packet_buf_counter = 0; //Amount of packets in buffer ^

data_packet wire_packet_buf[36];     //Packet buffer from THIS to F4 or ground control to F4
uint8_t wire_packet_buf_counter = 0; //Amount of packets in buffer ^

uint8_t ack_width = 0;
uint8_t ack_payload_test = 0;

uint32_t pps_timer = 0;

void setup()
{
  pinMode(PA6, OUTPUT);
  pinMode(PA7, OUTPUT);
  //pinMode(IRQ_pin, INPUT_PULLUP);

  digitalWrite(PA6, LOW); //Blue
  digitalWrite(PA7, LOW); //Green

  //pinMode(PB0, INPUT_ANALOG);
  //analogReadResolution(12);

  Serial.begin(115200);

  delay(5000);
  //NRF_Init();

  /*auto_packet_buf[0].id = GYRO_PACKET;
  PopulatePacketBuf(auto_packet_buf[0].payload, &gyro_x, 0);
  PopulatePacketBuf(auto_packet_buf[0].payload, &gyro_y, 2);
  PopulatePacketBuf(auto_packet_buf[0].payload, &gyro_z, 4);
  auto_packet_buf[0].width = 7; //3 * sizeof(int16_t) + 1(byte for id)*/
  auto_packet_count = 0;

  //Serial3.begin(9600);

  delay(200);
  //Setup_GPS();
  delay(200);
  //Serial2.end();

  //pps_timer = millis();

  //Wire.setSDA(PB7);
  //Wire.setSCL(PB6);
  //Wire.begin(MY_ADDRESS);

  //Wire.onReceive(I2C_Receive);
  //Wire.onRequest(I2C_Request);

  //battery_read_timer = millis();
  //is_calibrating_timer = millis();//???????????????????????????????????
  debug_i2c_timer = millis();
  debug_loop_timer = millis();
}

int16_union test_union;

uint8_t temp_led_state1 = 0;

void loop()
{
  if(millis() - debug_i2c_timer > 100)
  {
    digitalWrite(PA6, HIGH);
    Serial.println("YEET");
  }
  else
  {
    digitalWrite(PA6, LOW);
  }

  if(millis() - debug_loop_timer > 200)
  {
    debug_loop_timer = millis();
    temp_led_state1 = (~temp_led_state1) & 0x01;
    digitalWrite(PA7, temp_led_state1);
  }
  

  if(is_calibrating == 1)
  {
    if(millis() - is_calibrating_timer >= 11000)
    {
      is_calibrating = 0;
    }
  }

  
  if(millis() - battery_read_timer >= 1000)
  {
    battery_read_timer = millis();
    //ReadBatteryVoltage();
  }//????????????????????????????????????????????????????????

  //radio_loop();

  //Read_GPS();

  /*if (millis() - pps_timer >= 1000)
  {
    pps_timer = millis();

    Serial.print("Sent: ");
    Serial.print(sent_packet_counter);
    Serial.print(" ACKs: ");
    Serial.print(ack_packet_counter);
    Serial.print(" X: ");
    Serial.print(gyro_x);
    Serial.print(" Y: ");
    Serial.print(gyro_y);
    Serial.print(" Z: ");
    Serial.println(gyro_z);

    sent_packet_counter = 0;
    ack_packet_counter = 0;
  }*/


  sent_packet_counter = 0;//????
  ack_packet_counter = 0;//????

  if (i2c_receive_flag)
  {
    i2c_receive_flag = false;

    if (packet_buf_counter < 31)
    {
      for (int i = 0; i < wire_buf[39]; i++)
      {
        packet_buf[packet_buf_counter].payload[i] = wire_buf[i];
      }

      packet_buf[packet_buf_counter].width = wire_buf[39];

      packet_buf_counter++;
    }
  }
}

/*void I2C_Receive(int howMany)
{
  debug_i2c_timer = millis();
  Wire.readBytes((uint8_t *)wire_buf, howMany);
  wire_buf[39] = (uint8_t)howMany;

  i2c_receive_flag = true;
}

/void I2C_Request()
{
  //digitalWrite(PA6, HIGH);

  if (wire_packet_buf_counter > 0 && 1 == 0)
  {
    //digitalWrite(PA7, HIGH);

    Wire.write(wire_packet_buf[0].payload, (size_t)32);

    if (wire_packet_buf_counter > 1)
    {
      for (int i = 0; i < wire_packet_buf_counter - 1; i++)
      {
        //packet_buf[i] = packet_buf[i + 1];
        wire_packet_buf[i].width = wire_packet_buf[i + 1].width;

        for (int j = 0; j < wire_packet_buf[i + 1].width; j++)
        {
          wire_packet_buf[i].payload[j] = wire_packet_buf[i + 1].payload[j];
        }
      }
    }

    wire_packet_buf_counter--;
  }
  else
  {
    for (int i = 0; i < 32; i++)
    {
      wire_packet_buf[0].payload[i] = 0x00;
    }

    Wire.write(wire_packet_buf[0].payload, (size_t)32);
  }
}*/

void ReadBatteryVoltage()
{
  battery_voltage = analogRead(PB0);
}