#include "Arduino.h"
#include "SPI.h"
#include "nrf24radio.h"
#include "telemdata.h"
#include "string.h"

SPIClass rfspi(MOSI_pin, MISO_pin, SCK_pin);

byte data_in[34];

/**
* Packets to be updated automatically
* 0: Gyro 
*/
data_packet_pointer packet_buf[32];
uint8_t packet_buf_counter = 0;

int sent_packet_counter = 0;
int ack_packet_counter = 0;

uint8_t ack_width = 0;
uint8_t ack_payload_test = 0;

uint64_t pps_timer = 0;

uint64_t packet_lag_timer;
uint32_t min_packet_lag = 2500;


void NRF_Init()
{
    pinMode(CE_pin, OUTPUT);
    pinMode(CSN_pin, OUTPUT);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);
    pinMode(SCK_pin, OUTPUT);
    rfspi.setBitOrder(MSBFIRST);
    rfspi.setDataMode(SPI_MODE0);
    rfspi.setClockDivider(SPI_CLOCK_DIV8);
    digitalWrite(CE_pin, HIGH);
    digitalWrite(CSN_pin, HIGH);
    rfspi.begin();

    NRF_Get_Address(7);
    NRF_Write_Bit(0, 0, 1); //register#, bit#, and value 0 or 1, ::  0,0,1 RX Mode
    NRF_Write_Bit(0, 1, 1); //register, bit, and value 0,1,1 PowerUP
    //NRF_Write_Bit(0, 4, 1); //RT Mask turns off the RT interrupt
    //NRF_Write_Bit(0, 5, 1); //TX Mask turns off the TX interrupt
    //NRF_Write_Bit(0, 6, 1);

    NRF_Write_Bit(4, 4, 1);
    NRF_Write_Bit(4, 5, 1);
    NRF_Write_Bit(4, 6, 0);
    NRF_Write_Bit(4, 7, 1);

    NRF_Write_Bit(4, 3, 0);
    NRF_Write_Bit(4, 2, 1);
    NRF_Write_Bit(4, 1, 0);
    NRF_Write_Bit(4, 0, 1);

    NRF_Write_Bit(6, 3, 0);
    NRF_Write_Bit(6, 5, 1); //250kbps

    NRF_Write_Bit(5, 6, 1);
    NRF_Write_Bit(5, 5, 1);
    NRF_Write_Bit(5, 4, 0);
    NRF_Write_Bit(5, 3, 1);
    NRF_Write_Bit(5, 2, 1);
    NRF_Write_Bit(5, 1, 1);
    NRF_Write_Bit(5, 0, 0);

    NRF_Write_Bit(29, 2, 1);
    NRF_Write_Bit(29, 1, 1);

    NRF_Write_Bit(28, 0, 1);

    digitalWrite(CSN_pin, LOW);
    data_in[0] = rfspi.transfer(B11100010); //flush RX
    digitalWrite(CSN_pin, HIGH);
    digitalWrite(CSN_pin, LOW);
    data_in[0] = rfspi.transfer(B11100001); //flush TX
    digitalWrite(CSN_pin, HIGH);

    NRF_Clear_Interrupts(); //clears any interrupts

    attachInterrupt(digitalPinToInterrupt(IRQ_pin), _RF24_IRQ, FALLING);

    packet_lag_timer = micros();
}

byte packet_width = 0;

char test_buf[12];

float test_angle = 0;

void radio_loop()
{
    if (waiting_for_ack == 0 && micros() - packet_lag_timer > min_packet_lag)
    {
        packet_lag_timer = micros();
        //packet_buf_counter++;

        //strcpy((char *)test_buf, "Hello World");
        NRF_Send_Packet(packet_buf[packet_buf_counter].payload, (uint8_t)packet_buf[packet_buf_counter].id, (uint8_t)packet_buf[packet_buf_counter].width - 1);

        packet_buf_counter++;

        if(packet_buf_counter >= packet_count)
            packet_buf_counter = 0;
    }

    if (radio_irq_flag)
    {
        radio_irq_flag = 0;

        NRF_Get_Address(7);
        if (bitRead(data_in[1], 5)) //TX success
        {
            ack_packet_counter++;
            waiting_for_ack = 0;

            if (bitRead(data_in[1], 6)) //RX success
            {
                digitalWrite(CSN_pin, LOW);
                data_in[0] = rfspi.transfer(B01100000); //get packet width
                packet_width = rfspi.transfer(B00000000);
                digitalWrite(CSN_pin, HIGH);

                //delayMicroseconds(100);

                digitalWrite(CSN_pin, LOW);
                //digitalWrite(CE_pin, LOW);
                data_in[0] = rfspi.transfer(B01100001); //read the payload

                for (int i = 0; i < packet_width; i++)
                {
                    received_data[i] = rfspi.transfer(B00000000);
                    ack_width = packet_width;
                }

                if(received_data[0] != 0x00)
                    radio_receive_flag = 1;

                digitalWrite(CSN_pin, HIGH);

                //delayMicroseconds(100);

                digitalWrite(CSN_pin, LOW);
                data_in[0] = rfspi.transfer(B11100010); //flush RX
                digitalWrite(CSN_pin, HIGH);

                NRF_Write_Bit(7, 6, 1);
            }

            NRF_Write_Bit(7, 5, 1);
        }

        NRF_Get_Address(7);
        if (bitRead(data_in[1], 4)) //Max retransmission(failed)
        {
            waiting_for_ack = 0;
            NRF_Write_Bit(7, 4, 1);
        }
    }
}

void NRF_Send_Packet(uint8_t *buf[32], uint8_t id, uint8_t buf_width)
{
    digitalWrite(CSN_pin, LOW);
    data_in[0] = rfspi.transfer(B11100001); //flush TX, get rid of anything that might be in there
    digitalWrite(CSN_pin, HIGH);

    delayMicroseconds(100);

    digitalWrite(CSN_pin, LOW);
    data_in[0] = rfspi.transfer(B10100000); //load TX payload

    data_in[1] = rfspi.transfer(id);

    for (int i = 0; i < buf_width; i++)
    {
        data_in[i + 2] = rfspi.transfer(*buf[i]);
    }

    digitalWrite(CSN_pin, HIGH);

    waiting_for_ack = 1;
    sent_packet_counter++;

    digitalWrite(CE_pin, LOW); //pull CE pin LOW
    delayMicroseconds(500);    //small delay
    NRF_Write_Bit(0, 0, 0);    //go into TX mode
    //delay(1);//small delay
    digitalWrite(CE_pin, HIGH);
}

void _RF24_IRQ()
{
    radio_irq_flag = 1;

    //NRF_ClearInterrupts();
}

void NRF_Clear_Interrupts()
{
    NRF_Get_Address(7); //RT interrupt
    if (bitRead(data_in[1], 4))
        NRF_Write_Bit(7, 4, 1);

    NRF_Get_Address(7); //TX interrupt
    if (bitRead(data_in[1], 5))
        NRF_Write_Bit(7, 5, 1);

    NRF_Get_Address(7); //RX interrupt
    if (bitRead(data_in[1], 6))
        NRF_Write_Bit(7, 6, 1);
}

void NRF_Get_Address(byte address)
{
    digitalWrite(CSN_pin, LOW);
    data_in[0] = rfspi.transfer(address);
    data_in[1] = rfspi.transfer(B00000000);
    digitalWrite(CSN_pin, HIGH);
}

void NRF_Write_Bit(byte address, byte bit_add, byte val)
{
    NRF_Get_Address(address); //first read out the register
    if (val == 1)             //if we want to write a one to the bit then set the bit in the register we read
        bitSet(data_in[1], bit_add);
    else
        bitClear(data_in[1], bit_add); //clear it if not

    digitalWrite(CSN_pin, LOW);                //now we'll write the modified data back in
    data_in[0] = rfspi.transfer(32 + address); //a write to a register adds 32
    data_in[1] = rfspi.transfer(data_in[1]);   //write the modified register
    digitalWrite(CSN_pin, HIGH);
}