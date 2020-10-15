#include "Arduino.h"
#include "gps.h"
#include "nrf24radio.h"
#include "telemdata.h"

HardwareSerial Serial2(PA3, PA2);

uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};

uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};

uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1};

uint8_t new_line = 0;
uint8_t message_counter = 0;
uint8_t nmea_message[100];

int32_t latitude = 1, longitude = 1;
uint8_t sat_count = 0;
bool gps_fix = false;
uint8_t new_gps_data = 0;

int32_union int32_u;

void Setup_GPS()
{
    Serial2.begin(9600);
    delay(250);
    Serial2.write(Disable_GPGSV, 11);
    delay(400);
    Serial2.write(Set_to_5Hz, 14);
    delay(400);
    Serial2.write(Set_to_57kbps, 28);
    delay(400);
    Serial2.begin(57600);
}

void Read_GPS()
{
    while (Serial2.available() > 0 && new_line == 0)
    {
        char c = Serial2.read();

        if (c == '$')
        {
            message_counter = 0;

            for (int i = 0; i < 99; i++)
            {
                nmea_message[i] = '-';
            }
        }
        else if (message_counter <= 99)
        {
            nmea_message[message_counter] = c;
            message_counter++;
        }

        if (c == '*')
            new_line = 1;
    }

    if (new_line == 1)
    {
        if (nmea_message[3] == 'L' && nmea_message[4] == 'L' && nmea_message[6] == ',') //No fix
        {
            gps_fix = false;
            sat_count = 0;
            latitude = 1;
            longitude = 1;
        }
        else if (nmea_message[3] == 'L' && nmea_message[4] == 'L') //Fix
        {
            gps_fix = true;
        }

        if (nmea_message[3] == 'G' && nmea_message[4] == 'A' && (nmea_message[43] == '1' || nmea_message[43] == '2'))
        {
            latitude = 0;

            latitude += ((int)nmea_message[18] - 48) * 10000000;
            latitude += ((int)nmea_message[19] - 48) * 1000000;
            latitude += ((int)nmea_message[21] - 48) * 100000;
            latitude += ((int)nmea_message[22] - 48) * 10000;
            latitude += ((int)nmea_message[23] - 48) * 1000;
            latitude += ((int)nmea_message[24] - 48) * 100;
            latitude += ((int)nmea_message[25] - 48) * 10;
            latitude /= (int32_t)6;
            latitude += ((int)nmea_message[16] - 48) * 100000000;
            latitude += ((int)nmea_message[17] - 48) * 10000000;
            latitude /= (int32_t)10;

            longitude = 0;

            longitude += ((int)nmea_message[32] - 48) * 10000000;
            longitude += ((int)nmea_message[33] - 48) * 1000000;
            longitude += ((int)nmea_message[35] - 48) * 100000;
            longitude += ((int)nmea_message[36] - 48) * 10000;
            longitude += ((int)nmea_message[37] - 48) * 1000;
            longitude += ((int)nmea_message[38] - 48) * 100;
            longitude += ((int)nmea_message[39] - 48) * 10;
            longitude /= (int32_t)6;
            longitude += ((int)nmea_message[29] - 48) * 1000000000;
            longitude += ((int)nmea_message[30] - 48) * 100000000;
            longitude += ((int)nmea_message[31] - 48) * 10000000;
            longitude /= (int32_t)10;

            if (nmea_message[27] == 'S')
                latitude *= -1;

            if (nmea_message[41] == 'W')
                longitude *= -1;

            sat_count = (uint8_t)((int)nmea_message[45] - 48) * (uint8_t)10;
            sat_count += (uint8_t)((int)nmea_message[46] - 48);

            wire_packet_buf[wire_packet_buf_counter].payload[0] = 0x08; //GPS_PACKET

            int32_u.num = latitude;

            wire_packet_buf[wire_packet_buf_counter].payload[1] = int32_u.data[0];
            wire_packet_buf[wire_packet_buf_counter].payload[2] = int32_u.data[1];
            wire_packet_buf[wire_packet_buf_counter].payload[3] = int32_u.data[2];
            wire_packet_buf[wire_packet_buf_counter].payload[4] = int32_u.data[3];

            int32_u.num = longitude;

            wire_packet_buf[wire_packet_buf_counter].payload[5] = int32_u.data[0];
            wire_packet_buf[wire_packet_buf_counter].payload[6] = int32_u.data[1];
            wire_packet_buf[wire_packet_buf_counter].payload[7] = int32_u.data[2];
            wire_packet_buf[wire_packet_buf_counter].payload[8] = int32_u.data[3];

            wire_packet_buf[wire_packet_buf_counter].payload[9] = sat_count;

            for (int i = 10; i < 32; i++)
            {
                wire_packet_buf[wire_packet_buf_counter].payload[i] = 0x00;
            }

            wire_packet_buf[wire_packet_buf_counter].width = 10;
            wire_packet_buf_counter++;

            new_gps_data = 1;
        }
        new_line = 0;
    }
}