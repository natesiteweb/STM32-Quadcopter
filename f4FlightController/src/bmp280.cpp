#include "Arduino.h"
#include "Wire.h"
#include "bmp280.h"

void write8(byte reg, byte value);
uint32_t read24(byte reg);
uint16_t read16_LE(byte reg);
int16_t readS16_LE(byte reg);
uint16_t read16(byte reg);

float bmp_calibration = 0;
float bmp_altitude = 0;

float total_bmp_altitude = 0;
float fast_bmp_altitude = 0;
float slow_bmp_altitude = 0;
float last_bmp_altitude = 0;

float bmp_over_time[20];
uint8_t bmp_reading_index = 0;

float pressure_difference = 0;

float bmp_setpoint = 2;

void Calculate_Baro_Altitude()
{
    total_bmp_altitude -= bmp_over_time[bmp_reading_index];
    bmp_over_time[bmp_reading_index] = readAltitude(1013.25, true) - bmp_calibration;
    total_bmp_altitude += bmp_over_time[bmp_reading_index];

    fast_bmp_altitude = (total_bmp_altitude / 4.00);
    slow_bmp_altitude = (slow_bmp_altitude * 0.900) + (fast_bmp_altitude * 0.100);

    pressure_difference = slow_bmp_altitude - fast_bmp_altitude;

    if (pressure_difference > 0.400)
        pressure_difference = 0.400;
    else if (pressure_difference < -0.400)
        pressure_difference = -0.400;

    if (pressure_difference > 0.120 || pressure_difference < -0.120)
        slow_bmp_altitude -= pressure_difference / 2.000;

    bmp_altitude = slow_bmp_altitude;
}

void Calibrate_BMP()
{
    bmp_calibration = 0;

    for (int i = 0; i < 30; i++)
    {
        bmp_calibration += (float)readAltitude(1013.25, true);
        delay(100);
    }

    bmp_calibration /= 30.00;
}

typedef struct
{
    uint16_t dig_T1; /**< dig_T1 cal register. */
    int16_t dig_T2;  /**<  dig_T2 cal register. */
    int16_t dig_T3;  /**< dig_T3 cal register. */

    uint16_t dig_P1; /**< dig_P1 cal register. */
    int16_t dig_P2;  /**< dig_P2 cal register. */
    int16_t dig_P3;  /**< dig_P3 cal register. */
    int16_t dig_P4;  /**< dig_P4 cal register. */
    int16_t dig_P5;  /**< dig_P5 cal register. */
    int16_t dig_P6;  /**< dig_P6 cal register. */
    int16_t dig_P7;  /**< dig_P7 cal register. */
    int16_t dig_P8;  /**< dig_P8 cal register. */
    int16_t dig_P9;  /**< dig_P9 cal register. */
} bmp280_calib_data;

bmp280_calib_data _bmp280_calib;
int32_t t_fine;

void readCoefficients()
{
    _bmp280_calib.dig_T1 = read16_LE(0x88);
    _bmp280_calib.dig_T2 = readS16_LE(0x8A);
    _bmp280_calib.dig_T3 = readS16_LE(0x8C);

    _bmp280_calib.dig_P1 = read16_LE(0x8E);
    _bmp280_calib.dig_P2 = readS16_LE(0x90);
    _bmp280_calib.dig_P3 = readS16_LE(0x92);
    _bmp280_calib.dig_P4 = readS16_LE(0x94);
    _bmp280_calib.dig_P5 = readS16_LE(0x96);
    _bmp280_calib.dig_P6 = readS16_LE(0x98);
    _bmp280_calib.dig_P7 = readS16_LE(0x9A);
    _bmp280_calib.dig_P8 = readS16_LE(0x9C);
    _bmp280_calib.dig_P9 = readS16_LE(0x9E);
}

void setSampling()
{
    write8(0xF5, 0x10);
    write8(0xF4, 0x57);
}

void write8(byte reg, byte value)
{
    Wire.beginTransmission(BMP_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}

int32_t adc_P;
int64_t pressureVar1, pressureVar2, pressureVarP;

float readPressure()
{
    // Must be done first to get the t_fine variable set up
    //readTemperature();

    adc_P = read24(0xF7);
    adc_P >>= 4;

    pressureVar1 = ((int64_t)t_fine) - 128000;
    pressureVar2 = pressureVar1 * pressureVar1 * (int64_t)_bmp280_calib.dig_P6;
    pressureVar2 = pressureVar2 + ((pressureVar1 * (int64_t)_bmp280_calib.dig_P5) << 17);
    pressureVar2 = pressureVar2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
    pressureVar1 = ((pressureVar1 * pressureVar1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
                   ((pressureVar1 * (int64_t)_bmp280_calib.dig_P2) << 12);
    pressureVar1 =
        (((((int64_t)1) << 47) + pressureVar1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

    if (pressureVar1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    pressureVarP = 1048576 - adc_P;
    pressureVarP = (((pressureVarP << 31) - pressureVar2) * 3125) / pressureVar1;
    pressureVar1 = (((int64_t)_bmp280_calib.dig_P9) * (pressureVarP >> 13) * (pressureVarP >> 13)) >> 25;
    pressureVar2 = (((int64_t)_bmp280_calib.dig_P8) * pressureVarP) >> 19;

    pressureVarP = ((pressureVarP + pressureVar1 + pressureVar2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
    return (float)pressureVarP / 256;
}

int32_t temperatureVar1, temperatureVar2, adc_T;
float tempTemperature;

float readTemperature()
{
    adc_T = read24(0xFA);
    adc_T >>= 4;

    temperatureVar1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
                       ((int32_t)_bmp280_calib.dig_T2)) >>
                      11;

    temperatureVar2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
                         ((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >>
                        12) *
                       ((int32_t)_bmp280_calib.dig_T3)) >>
                      14;

    t_fine = temperatureVar1 + temperatureVar2;

    tempTemperature = (t_fine * 5 + 128) >> 8;
    return tempTemperature / 100;
}

float localAlt, localPressure;

float readAltitude(float seaLevelhPa, bool readTemp)
{
    if (readTemp)
        readTemperature();

    localPressure = readPressure(); // in Si units for Pascal
    localPressure /= 100;

    localAlt = 44330 * (1.0 - pow(localPressure / seaLevelhPa, 0.1903));

    return localAlt;
}

uint32_t read24(byte reg)
{
    uint32_t value;

    Wire.beginTransmission((uint8_t)BMP_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)BMP_ADDRESS, (uint8_t)3);

    value = Wire.read();
    value <<= 8;
    value |= Wire.read();
    value <<= 8;
    value |= Wire.read();

    return value;
}

uint16_t read16_LE(byte reg)
{
    uint16_t temp = read16(reg);
    return (temp >> 8) | (temp << 8);
}

int16_t readS16_LE(byte reg)
{
    return (int16_t)read16_LE(reg);
}

uint16_t read16(byte reg)
{
    uint16_t value;
    Wire.beginTransmission(BMP_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(BMP_ADDRESS, (uint8_t)2);
    value = (Wire.read() << 8) | Wire.read();

    return value;
}