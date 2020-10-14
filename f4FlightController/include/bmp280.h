#ifndef _BMP280_H
#define _BMP280_H

void Calculate_Baro_Altitude(void);
void readCoefficients(void);
void setSampling(void);
void Calibrate_BMP(void);

float readAltitude(float seaLevelhPa, bool readTemp);
float readTemperature();
float readPressure();

extern uint8_t BMP_ADDRESS;

extern float bmp_calibration;
extern float bmp_altitude;
extern uint8_t bmp_reading_index;

extern float bmp_setpoint;

#endif