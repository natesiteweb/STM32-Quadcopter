#include "Arduino.h"
#include "Wire.h"
#include "wiretelem.h"
#include "eepromi2c.h"
#include "pid_logic.h"
#include "imu.h"
#include "bmp280.h"
#include "compass.h"

TwoWire Wire2(PB11, PB10);

//Packets to automatically send
data_packet_pointer auto_packet_buf[32];
uint8_t auto_packet_buf_counter = 0;

//Manual Packets
data_packet packet_buf[32];
uint8_t packet_buf_counter = 0;

uint8_t manual_packet_index = 0;

uint8_t auto_packet_count = 0; //How many automatic packets there are

uint8_t telem_rate = 2;       //Every program loop (2 means every other program loop)
uint8_t telem_read_rate = 10; //This * telem_rate is actual read rate (40ms)

uint8_t telem_rate_counter = 0;
uint8_t telem_read_counter = 0;

uint8_t wire_receive_data[40];

int32_union int32_u;
uint32_union uint32_u;

void telem_wire_setup()
{
    Wire2.setClock((uint32_t)400000);
    Wire2.begin();
}

uint8_t temp_led_state = 0;
uint8_t gps_read_index;

void telem_loop()
{
    telem_rate_counter++;

    if (telem_rate_counter >= telem_rate)
    {
        telem_rate_counter = 0;

        telem_read_counter++;
        telem_timer = millis();

        if (telem_read_counter >= telem_read_rate)
        {
            telem_read_counter = 0;

            Wire2.requestFrom((uint8_t)0x04, (uint8_t)32);

            //delayMicroseconds(500);

            Wire2.readBytes((uint8_t *)wire_receive_data, (size_t)32);

            switch (wire_receive_data[0])
            {
            case 0x00:
                break;
            case 0xF7:
                temp_led_state = (~temp_led_state) & 0x01;
                digitalWrite(PC2, temp_led_state);
                flight_mode = 1;

                //TelemPrintDebug((char *)"Calibrated.\n", 12);

                CalibrateIMU();
                Calibrate_BMP();
                ResetTimers();
                break;
            case PID_GAIN_FIRST_REQUEST:
                packet_buf[packet_buf_counter].id = PID_GAIN_FIRST_PACKET;
                manual_packet_index = 0;
                PopulateManualPacket(kp_roll);
                PopulateManualPacket(ki_roll);
                PopulateManualPacket(kd_roll);
                PopulateManualPacket(kp_yaw);
                PopulateManualPacket(ki_yaw);
                PopulateManualPacket(kd_yaw);
                packet_buf[packet_buf_counter].width = manual_packet_index + 1;
                packet_buf_counter++;
                break;
            case PID_GAIN_FIRST_UPDATE_REQUEST:
                manual_packet_index = 1;
                kp_roll = ReadManualPacket_Float();
                ki_roll = ReadManualPacket_Float();
                kd_roll = ReadManualPacket_Float();
                kp_yaw = ReadManualPacket_Float();
                ki_yaw = ReadManualPacket_Float();
                kd_yaw = ReadManualPacket_Float();

                kp_pitch = kp_roll;
                ki_pitch = ki_roll;
                kd_pitch = kd_roll;

                EEPROM_Clear_Buf();
                EEPROM_Float_Write(0, kp_roll);
                EEPROM_Float_Write(4, ki_roll);
                EEPROM_Float_Write(8, kd_roll);
                EEPROM_Float_Write(12, kp_yaw);
                EEPROM_Float_Write(16, ki_yaw);
                EEPROM_Float_Write(20, kd_yaw);
                EEPROM_Save_Page(0);

                packet_buf[packet_buf_counter].id = PID_GAIN_FIRST_PACKET;
                manual_packet_index = 0;
                PopulateManualPacket(kp_roll);
                PopulateManualPacket(ki_roll);
                PopulateManualPacket(kd_roll);
                PopulateManualPacket(kp_yaw);
                PopulateManualPacket(ki_yaw);
                PopulateManualPacket(kd_yaw);
                packet_buf[packet_buf_counter].width = manual_packet_index + 1;
                packet_buf_counter++;
                break;
            case PID_GAIN_SECOND_REQUEST:
                packet_buf[packet_buf_counter].id = PID_GAIN_SECOND_PACKET;
                manual_packet_index = 0;
                PopulateManualPacket(kp_altitude);
                PopulateManualPacket(ki_altitude);
                PopulateManualPacket(kd_altitude);
                PopulateManualPacket(kp_gps);
                PopulateManualPacket(ki_gps);
                PopulateManualPacket(kd_gps);
                packet_buf[packet_buf_counter].width = manual_packet_index + 1;
                packet_buf_counter++;
                break;
            case PID_GAIN_SECOND_UPDATE_REQUEST:
                manual_packet_index = 1;
                kp_altitude = ReadManualPacket_Float();
                ki_altitude = ReadManualPacket_Float();
                kd_altitude = ReadManualPacket_Float();
                kp_gps = ReadManualPacket_Float();
                ki_gps = ReadManualPacket_Float();
                kd_gps = ReadManualPacket_Float();

                EEPROM_Clear_Buf();
                EEPROM_Float_Write(0, kp_altitude);
                EEPROM_Float_Write(4, ki_altitude);
                EEPROM_Float_Write(8, kd_altitude);
                EEPROM_Float_Write(12, kp_gps);
                EEPROM_Float_Write(16, ki_gps);
                EEPROM_Float_Write(20, kd_gps);
                EEPROM_Save_Page(32);

                packet_buf[packet_buf_counter].id = PID_GAIN_SECOND_PACKET;
                manual_packet_index = 0;
                PopulateManualPacket(kp_altitude);
                PopulateManualPacket(ki_altitude);
                PopulateManualPacket(kd_altitude);
                PopulateManualPacket(kp_gps);
                PopulateManualPacket(ki_gps);
                PopulateManualPacket(kd_gps);
                packet_buf[packet_buf_counter].width = manual_packet_index + 1;
                packet_buf_counter++;
                break;
            case CALIBRATE_ESC_REQUEST:
                EEPROM_Single_Byte_Write(0x8000, 0x01);
                //delay(1000);
                break;
            case ALTITUDE_REQUEST: //If you click download on ground control
                packet_buf[packet_buf_counter].id = ALTITUDE_SET_PACKET;
                manual_packet_index = 0;
                PopulateManualPacket(bmp_setpoint);
                packet_buf[packet_buf_counter].width = manual_packet_index + 1;
                packet_buf_counter++;
                break;
            case ALTITUDE_SET_REQUEST: //If you upload altitude setpoint
                manual_packet_index = 1;
                bmp_setpoint = ReadManualPacket_Float();

                packet_buf[packet_buf_counter].id = ALTITUDE_SET_PACKET;
                manual_packet_index = 0;
                PopulateManualPacket(bmp_setpoint);
                packet_buf[packet_buf_counter].width = manual_packet_index + 1;
                packet_buf_counter++;
                break;
            case GPS_PACKET:
                manual_packet_index = 1;
                raw_latitude = ReadManualPacket_Int32();
                raw_longitude = ReadManualPacket_Int32();
                sat_count = wire_receive_data[manual_packet_index];
                manual_packet_index++;

                battery_voltage = ((float)ReadManualPacket_UInt32()) * (float)0.004592;

                new_gps_data = 1;
                latitude_table[0] = raw_latitude;
                longitude_table[0] = raw_longitude;

                if (flight_mode < 4)
                {
                    latitude_table[2] = raw_latitude;
                    longitude_table[2] = raw_longitude;
                }

                lat_setpoint = latitude_table[2];
                lon_setpoint = longitude_table[2];

                calculated_lat_error = (float)(raw_latitude - lat_setpoint);
                calculated_lon_error = (float)(lon_setpoint - raw_longitude);

                lat_add = (float)(raw_latitude - last_raw_latitude) / (float)10;
                lon_add = (float)(last_raw_longitude - raw_longitude) / (float)10;

                last_raw_latitude = raw_latitude;
                last_raw_longitude = raw_longitude;

                SendGPSPacket((uint8_t)0);
                break;
            case CALIBRATE_COMPASS_REQUEST:
                flight_mode = 1;

                //TelemPrintDebug((char *)"Calibrated Compass.\n", 20);

                CalibrateCompass();
                ResetTimers();
                break;
            case GPS_PACKET_UPDATE_REQUEST:
                manual_packet_index = 1;
                gps_read_index = wire_receive_data[manual_packet_index];
                manual_packet_index++;

                if (gps_read_index > 4)
                {
                    latitude_table[gps_read_index] = ReadManualPacket_Int32();
                    longitude_table[gps_read_index] = ReadManualPacket_Int32();
                }

                break;
            case GPS_PACKET_REQUEST: //Request hold position
                SendGPSPacket((uint8_t)5);
                break;
            case GPS_HOLD_COPY_BUFFER_REQUEST: //Copy hold position buffer
                latitude_table[3] = latitude_table[5];
                longitude_table[3] = longitude_table[5];

                latitude_table[4] = latitude_table[2];
                longitude_table[4] = longitude_table[2];

                lat_modifier_add = 0;
                lon_modifier_add = 0;

                lat_modifier = 0;
                lon_modifier = 0;
                break;
            }
        }
        else if (packet_buf_counter > 0)
        {
            Wire2.beginTransmission(0x04);
            Wire2.write(packet_buf[0].id);

            for (int i = 0; i < packet_buf[0].width - 1; i++)
            {
                Wire2.write(packet_buf[0].payload[i]);
            }

            Wire2.endTransmission();

            if (packet_buf_counter > 1)
            {
                for (int i = 0; i < packet_buf_counter - 1; i++)
                {
                    packet_buf[i].width = packet_buf[i + 1].width;

                    for (int j = 0; j < packet_buf[i + 1].width; j++)
                    {
                        packet_buf[i].payload[j] = packet_buf[i + 1].payload[j];
                    }
                }
            }

            packet_buf_counter--;
        }
        else if (auto_packet_count > 0)
        {
            Wire2.beginTransmission(0x04);
            Wire2.write(auto_packet_buf[auto_packet_buf_counter].id);

            for (int i = 0; i < auto_packet_buf[auto_packet_buf_counter].width - 1; i++)
            {
                Wire2.write(*auto_packet_buf[auto_packet_buf_counter].payload[i]);
            }

            Wire2.endTransmission();

            auto_packet_buf_counter++;

            if (auto_packet_buf_counter >= auto_packet_count)
                auto_packet_buf_counter = 0;
        }
    }
}

void TelemPrintDebug(char txt_to_print[30], uint8_t message_length)
{
    packet_buf[packet_buf_counter].id = PRINT_PACKET;
    manual_packet_index = 0;

    PopulateManualPacket(message_length);

    for (int i = 0; i < message_length; i++)
    {
        PopulateManualPacket(((uint8_t)txt_to_print[i]));
    }

    packet_buf[packet_buf_counter].width = manual_packet_index + 1;
    packet_buf_counter++;
}

void SendGPSPacket(uint8_t gps_index)
{
    packet_buf[packet_buf_counter].id = GPS_PACKET;
    manual_packet_index = 0;
    PopulateManualPacket((uint8_t)gps_index);
    PopulateManualPacket((uint8_t)sat_count);
    PopulateManualPacket((int32_t)latitude_table[gps_index]);
    PopulateManualPacket((int32_t)longitude_table[gps_index]);
    packet_buf[packet_buf_counter].width = manual_packet_index + 1;
    packet_buf_counter++;
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

void PopulatePacketBuf(uint8_t **buf, int32_t *num, int start_index)
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

void PopulatePacketBuf(uint8_t **buf, uint8_t *num, int start_index)
{
    buf[start_index] = (uint8_t *)num;
}

void PopulateManualPacket(float num)
{
    float_u.num = num;

    packet_buf[packet_buf_counter].payload[manual_packet_index] = float_u.data[0];
    packet_buf[packet_buf_counter].payload[manual_packet_index + 1] = float_u.data[1];
    packet_buf[packet_buf_counter].payload[manual_packet_index + 2] = float_u.data[2];
    packet_buf[packet_buf_counter].payload[manual_packet_index + 3] = float_u.data[3];

    manual_packet_index += 4;
}

void PopulateManualPacket(int32_t num)
{
    int32_u.num = num;

    packet_buf[packet_buf_counter].payload[manual_packet_index] = int32_u.data[0];
    packet_buf[packet_buf_counter].payload[manual_packet_index + 1] = int32_u.data[1];
    packet_buf[packet_buf_counter].payload[manual_packet_index + 2] = int32_u.data[2];
    packet_buf[packet_buf_counter].payload[manual_packet_index + 3] = int32_u.data[3];

    manual_packet_index += 4;
}

void PopulateManualPacket(uint8_t num)
{
    packet_buf[packet_buf_counter].payload[manual_packet_index] = num;

    manual_packet_index++;
}

float ReadManualPacket_Float()
{
    float_u.data[0] = wire_receive_data[manual_packet_index];
    float_u.data[1] = wire_receive_data[manual_packet_index + 1];
    float_u.data[2] = wire_receive_data[manual_packet_index + 2];
    float_u.data[3] = wire_receive_data[manual_packet_index + 3];

    manual_packet_index += 4;

    return float_u.num;
}

int32_t ReadManualPacket_Int32()
{
    int32_u.data[0] = wire_receive_data[manual_packet_index];
    int32_u.data[1] = wire_receive_data[manual_packet_index + 1];
    int32_u.data[2] = wire_receive_data[manual_packet_index + 2];
    int32_u.data[3] = wire_receive_data[manual_packet_index + 3];

    manual_packet_index += 4;

    return int32_u.num;
}

uint32_t ReadManualPacket_UInt32()
{
    uint32_u.data[0] = wire_receive_data[manual_packet_index];
    uint32_u.data[1] = wire_receive_data[manual_packet_index + 1];
    uint32_u.data[2] = wire_receive_data[manual_packet_index + 2];
    uint32_u.data[3] = wire_receive_data[manual_packet_index + 3];

    manual_packet_index += 4;

    return uint32_u.num;
}