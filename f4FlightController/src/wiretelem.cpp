#include "Arduino.h"
#include "Wire.h"
#include "wiretelem.h"

TwoWire Wire2(PB11, PB10);

//Packets to automatically send
data_packet_pointer packet_buf[32];
uint8_t packet_buf_counter = 0;

uint8_t packet_count = 0; //How many automatic packets there are

void telem_wire_setup()
{
    Wire2.setClock((uint32_t)400000);
    Wire2.begin();
}

void telem_loop()
{
    if (millis() - telem_timer > 5)
    {
        telem_timer = millis();

        Wire2.beginTransmission(0x04);
        Wire2.write(packet_buf[packet_buf_counter].id);

        for (int i = 0; i < packet_buf[packet_buf_counter].width - 1; i++)
        {
            Wire2.write(*packet_buf[packet_buf_counter].payload[i]);
        }

        Wire2.endTransmission();

        packet_buf_counter++;

        if (packet_buf_counter >= packet_count)
            packet_buf_counter = 0;
    }
}

void PopulatePacketBuf(uint8_t **buf, float *num, int start_index)
{
    buf[start_index] = (uint8_t *)num;
    buf[start_index + 1] = ((uint8_t *)num) + 1;
    buf[start_index + 2] = ((uint8_t *)num) + 2;
    buf[start_index + 3] = ((uint8_t *)num) + 3;
}

void PopulatePacketBuf(uint8_t **buf, uint32_t *num, int start_index)
{
    buf[start_index] = (uint8_t *)num;
    buf[start_index + 1] = ((uint8_t *)num) + 1;
    buf[start_index + 2] = ((uint8_t *)num) + 2;
    buf[start_index + 3] = ((uint8_t *)num) + 3;
}

void PopulatePacketBuf(uint8_t **buf, uint16_t *num, int start_index)
{
    buf[start_index] = (uint8_t *)num;
    buf[start_index + 1] = ((uint8_t *)num) + 1;
}

void PopulatePacketBuf(uint8_t **buf, int16_t *num, int start_index)
{
    buf[start_index] = (uint8_t *)num;
    buf[start_index + 1] = ((uint8_t *)num) + 1;
}