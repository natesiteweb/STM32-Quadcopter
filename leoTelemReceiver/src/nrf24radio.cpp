#include "Arduino.h"
#include "SPI.h"
#include "nrf24radio.h"

byte data_in[33];
uint8_t payload_width = 32;

uint8_t last_ack_index = 0;

volatile uint8_t radio_irq_flag = 0;

void NRF_Init()
{
    pinMode(CE_pin, OUTPUT);
    pinMode(CSN_pin, OUTPUT);
    pinMode(IRQ_pin, INPUT_PULLUP);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    digitalWrite(CE_pin, HIGH);
    digitalWrite(CSN_pin, HIGH);
    SPI.begin();

    NRF_Write_Bit(0, 0, 1); //register#, bit#, and value 0 or 1, ::  0,0,1 RX Mode
    NRF_Write_Bit(0, 1, 1); //register, bit, and value 0,1,1 PowerUP
    NRF_Write_Bit(0, 4, 1); //RT Mask turns off the RT interrupt
    NRF_Write_Bit(0, 5, 1); //TX Mask turns off the TX interrupt
                            //NRFwrite_bit_write(0, 6, 1);//RX Mask turns off the RX interrupt

    NRF_Write_Byte(4, B11110101); //Highest 4 bits are retry length, lowest are max retry counter

    NRF_Write_Bit(6, 3, 0);
    NRF_Write_Bit(6, 5, 1); //250kbps

    NRF_Write_Byte(5, B1101110); //Channel 110

    NRF_Write_Bit(29, 2, 1); //Enable Dynamic Payload Length
    NRF_Write_Bit(29, 1, 1); //Enable ACK with Payload
    NRF_Write_Bit(29, 0, 1); //Enable Payloads without ACKs

    NRF_Write_Bit(28, 0, 1); //Enable dynamic payload length on pipe 0

    NRF_Flush_RX();
    NRF_Flush_TX();

    NRF_Clear_Interrupts(); //clears any interrupts

    attachInterrupt(digitalPinToInterrupt(IRQ_pin), _RECEIVE, FALLING);

    NRF_Set_Ack_Payload();
}

void get_data()
{ // get data start get data start get data start get data start get data start
    // this routine is called when the IRQ pin is pulled LOW by the NRF

    //int i;

    if((last_ack_index != 0 && ack_payload_buf_counter == 0) || ack_payload_buf_counter > 0)
    {
        NRF_Set_Ack_Payload();
    }

    NRF_Get_Packet_Width();

    digitalWrite(CSN_pin, LOW);
    //digitalWrite(CE_pin, LOW);
    data_in[0] = SPI.transfer(B01100001); //read the payload

    for (int i = 0; i < payload_width; i++)
    {
        received_data[i] = SPI.transfer(B00000000);
    }

    digitalWrite(CSN_pin, HIGH);

    received_data_len = payload_width;

    digitalWrite(CSN_pin, LOW);
    data_in[0] = SPI.transfer(B11100010); //flush RX
    digitalWrite(CSN_pin, HIGH);

    //digitalWrite(CE_pin, HIGH);

    //NRFwrite_bit_write(7, 6, 1); //clear the RX interrupt flag

} //     END   get  data     END   get  data     END   get  data     END   get  data     END   get  data

uint8_t current_ack_counter;

void NRF_Set_Ack_Payload()
{
    //NRF_Flush_TX();

    if (ack_payload_buf_counter > 0)
    {
        current_ack_counter = 1;
        last_ack_index = 1;
    }
    else
    {
        current_ack_counter = 0;
    }

    last_ack_index = current_ack_counter;

    digitalWrite(CSN_pin, LOW);
    data_in[0] = SPI.transfer(B10101000);

    for (int i = 0; i < ack_payload_buf[current_ack_counter].width; i++)
    {
        data_in[i + 1] = SPI.transfer((byte)ack_payload_buf[current_ack_counter].payload[i]);
    }

    if (ack_payload_buf_counter > 0)
    {
        ack_payload_buf_counter--;

        for (int i = 1; i < 31; i++)
        {
            ack_payload_buf[i] = ack_payload_buf[i + 1];
        }
    }

    digitalWrite(CSN_pin, HIGH);
}

void _RECEIVE()
{
    radio_irq_flag = 1;
}

void NRF_Get_Packet_Width()
{
    digitalWrite(CSN_pin, LOW);
    data_in[0] = SPI.transfer(B01100000); //get packet width
    payload_width = SPI.transfer(B00000000);
    digitalWrite(CSN_pin, HIGH);
}

void NRF_Clear_Interrupts()
{ //    start clear interrupts      start clear interrupts      start clear interrupts
    //there are three interrupt flags in the NRF.  Thsi routine checks them, and if set, it will clear them

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
    data_in[0] = SPI.transfer(address);
    data_in[1] = SPI.transfer(B00000000);
    digitalWrite(CSN_pin, HIGH);
}

void NRF_Write_Bit(byte address, byte bit_add, byte val)
{
    NRF_Get_Address(address); //first read out the register
    if (val == 1)             //if we want to write a one to the bit then set the bit in the register we read
        bitSet(data_in[1], bit_add);
    else
        bitClear(data_in[1], bit_add); //clear it if not

    NRF_Write_Byte(address, data_in[1]);
}

void NRF_Write_Byte(byte address, byte val)
{
    digitalWrite(CSN_pin, LOW);
    data_in[0] = SPI.transfer(32 + address);
    data_in[1] = SPI.transfer(val);
    digitalWrite(CSN_pin, HIGH);
}

void NRF_Flush_TX()
{
    digitalWrite(CSN_pin, LOW);
    data_in[0] = SPI.transfer(B11100001); //flush TX
    digitalWrite(CSN_pin, HIGH);
}

void NRF_Flush_RX()
{
    digitalWrite(CSN_pin, LOW);
    data_in[0] = SPI.transfer(B11100010); //flush RX
    digitalWrite(CSN_pin, HIGH);
}