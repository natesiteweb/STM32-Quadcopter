#include "Arduino.h"
#include "Wire.h"
#include "compass.h"
#include "imu.h"
#include "eepromi2c.h"

uint32_t calibrate_compass_timer;

float compass_heading;

int16_t compass_x_min = 0, compass_x_max = 0;
int16_t compass_y_min = 0, compass_y_max = 0;
int16_t compass_z_min = 0, compass_z_max = 0;

int16_t compassX = 0;
int16_t compassY = 0;
int16_t compassZ = 0;

float comX = 0;
float comY = 0;
float comZ = 0;
float y_scale;
float z_scale;

float compass_offset_x = 0;
float compass_offset_y = 0;
float compass_offset_z = 0;

float heading_difference_return = 0;

void SetupCompass()
{
    Wire.beginTransmission(COMPASS_ADDRESS);
    Wire.write(0x00);
    Wire.write(0x78);
    Wire.write(0x20);
    Wire.write(0x00);
    Wire.endTransmission();

    CalculateCompassCalibration();

    delay(5);

    ReadCompass();

    total_gyro_z_angle = compass_heading;
}

void ReadCompass()
{
    Wire.beginTransmission(COMPASS_ADDRESS);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(COMPASS_ADDRESS, (uint8_t)6);

    compassY = (Wire.read() << 8) | Wire.read();
    compassZ = (Wire.read() << 8) | Wire.read();
    compassX = (Wire.read() << 8) | Wire.read();

    compassY += compass_offset_y;
    compassY *= y_scale;

    compassZ += compass_offset_z;
    compassZ *= z_scale;

    compassX += compass_offset_x;

    comX = ((float)compassX * cos((pitch_angle + 3.00) * 0.0174533)) + ((float)compassY * sin((roll_angle + 2.00) * 0.0174533) * sin((pitch_angle + 3.00) * 0.0174533)) - ((float)compassZ * cos((roll_angle + 2.00) * 0.0174533) * sin((pitch_angle + 3.00) * 0.0174533));

    comY = ((float)compassY * cos((roll_angle + 2.00) * 0.0174533)) + ((float)compassZ * sin((roll_angle + 2.00) * 0.0174533));

    if (comY < 0)
        compass_heading = 180 + (180 + ((atan2((float)comY, (float)comX)) * 57.29577));
    else
        compass_heading = atan2((float)comY, (float)comX) * 57.29577;

    compass_heading = -compass_heading;

    compass_heading -= 5;

    if (compass_heading < 0)
        compass_heading += 360;
    else if (compass_heading >= 360)
        compass_heading -= 360;
}

void CalibrateCompass()
{
    compass_x_min = 0;
    compass_x_max = 0;
    compass_y_min = 0;
    compass_y_max = 0;
    compass_z_min = 0;
    compass_z_max = 0;

    calibrate_compass_timer = millis();

    while (millis() - calibrate_compass_timer < 8000)
    {
        Wire.beginTransmission(COMPASS_ADDRESS);
        Wire.write(0x03);
        Wire.endTransmission();
        Wire.requestFrom(COMPASS_ADDRESS, (uint8_t)6);

        compassY = (Wire.read() << 8) | Wire.read();
        compassZ = (Wire.read() << 8) | Wire.read();
        compassX = (Wire.read() << 8) | Wire.read();

        if (compassX > compass_x_max)
            compass_x_max = compassX;
        else if (compassX < compass_x_min)
            compass_x_min = compassX;

        if (compassZ > compass_z_max)
            compass_z_max = compassZ;
        else if (compassZ < compass_z_min)
            compass_z_min = compassZ;

        if (compassY > compass_y_max)
            compass_y_max = compassY;
        else if (compassY < compass_y_min)
            compass_y_min = compassY;

        delay(5);
    }

    EEPROM_Clear_Buf();
    EEPROM_Int16_Write(0, compass_x_min);
    EEPROM_Int16_Write(2, compass_x_max);
    EEPROM_Int16_Write(4, compass_y_min);
    EEPROM_Int16_Write(6, compass_y_max);
    EEPROM_Int16_Write(8, compass_z_min);
    EEPROM_Int16_Write(10, compass_z_max);
    EEPROM_Save_Page(64);

    CalculateCompassCalibration();

    ReadCompass();

    total_gyro_z_angle = compass_heading;
}

void CalculateCompassCalibration()
{
    y_scale = (float)(compass_x_max - compass_x_min) / (float)(compass_y_max - compass_y_min);
    z_scale = (float)(compass_x_max - compass_x_min) / (float)(compass_z_max - compass_z_min);

    compass_offset_x = (float)(compass_x_max - compass_x_min) / 2 - compass_x_max;
    compass_offset_y = ((float)(compass_y_max - compass_y_min) / 2 - compass_y_max) * y_scale;
    compass_offset_z = ((float)(compass_z_max - compass_z_min) / 2 - compass_z_max) * z_scale;
}

void CalculateHeadingDifference(float ang1, float ang2) //ang1 is setpoint, ang2 is current
{
    heading_difference_return = ang1 - ang2;

    if (heading_difference_return < -180 || heading_difference_return > 180)
    {
        if (ang2 > 180)
            ang2 -= 180;
        else
            ang2 += 180;

        if (ang1 > 180)
            ang1 -= 180;
        else
            ang1 += 180;

        heading_difference_return = ang1 - ang2;
    }
}