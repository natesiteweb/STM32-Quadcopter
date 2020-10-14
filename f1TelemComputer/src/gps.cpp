#include "Arduino.h"
#include "gps.h"

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

        if()
    }
}