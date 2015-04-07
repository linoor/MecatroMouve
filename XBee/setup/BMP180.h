#include <Arduino.h>
#include <Wire.h>
#include "I2C.h"

// BMP180 I2C address
#define BMP180_ADDRESS    0x77

// BMP registers addresses
#define BMP180_OUT_XLSB   0xF8
#define BMP180_OUT_LSB    0xF7
#define BMP180_OUT_MSB    0xF6
#define BMP180_CTRL_MEAS  0xF4
#define BMP180_SOFT       0xE0
#define BMP180_ID         0xD0
#define BMP180_CALIB      0xAA

// **********************************************************************
// BMP180 CONFIGURATION FUNCTIONS
// **********************************************************************
// ************************************************************
// Function BMP180_Init
// Action : Initialize BMP180
// Argument : OverSampling rate (between 0 and 3)
// OverSample value  Samples Number  Acquisition Time (ms)
//        0                1                4.5
//        1                2                7.5
//        2                4                13.5
//        3                8                25.5
void BMP180_Init(byte OverSample);
// ************************************************************
// Function BMP180_readRawPressure()
// Action : Read raw pressure registers
// Return value : Raw pressure measurement
long BMP180_readRawPressure();

// ************************************************************
// Function BMP180_readRawTemperature()
// Action : Read raw temperature registers
// Return value : Raw temperature measurement
long BMP180_readRawTemperature();

// ************************************************************
// Function BMP180_getMeasurements(temperature, pressure)
// Action : Calculate temperature in °C, pressure in Pascal
// Arguments :
//    temperature : Variable which will store the temperature value (°C)
//    pressure : Variable which will store the pressure value (Pa)
//    altitude : Variable which will store the altitude value (m)
//    Arguments are float type
void BMP180_getMeasurements(float &Temperature, float &Pressure, float &Altitude);

