#include "BMP180.h"

// **********************************************************************
// Global variables
// **********************************************************************
// Oversample rate
byte OSS;
int AC1, AC2, AC3, BB1, BB2, MB, MC, MD;
unsigned int AC4, AC5, AC6;

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
void BMP180_Init(byte OverSample)
{
  byte regData;

  if (OverSample > 3) OverSample = 3;
  OSS = OverSample;

  Wire.begin();      // Initialize I2C bus as a master

  // Set OverSample value
  regData = I2C_Read(BMP180_ADDRESS, BMP180_CTRL_MEAS);
  regData &= 0x3F;
  regData |= (OverSample << 6);
  I2C_Write(BMP180_ADDRESS, BMP180_CTRL_MEAS, regData);

  // Read calibration coefficients
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(BMP180_CALIB);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP180_ADDRESS, 22);
  while (Wire.available() < 22);
  AC1 = (int(Wire.read()) << 8) | Wire.read();
  AC2 = (int(Wire.read()) << 8) | Wire.read();
  AC3 = (int(Wire.read()) << 8) | Wire.read();
  AC4 = (word(Wire.read()) << 8) | Wire.read();
  AC5 = (word(Wire.read()) << 8) | Wire.read();
  AC6 = (word(Wire.read()) << 8) | Wire.read();
  BB1 = (int(Wire.read()) << 8) | Wire.read();
  BB2 = (int(Wire.read()) << 8) | Wire.read();
  MB = (int(Wire.read()) << 8) | Wire.read();
  MC = (int(Wire.read()) << 8) | Wire.read();
  MD = (int(Wire.read()) << 8) | Wire.read();
}

// ************************************************************
// Function BMP180_readRawPressure()
// Action : Read raw pressure registers
// Return value : Raw pressure measurement
long BMP180_readRawPressure()
{
  long pressure;
  byte regData;

  // Start a pressure measurement
  regData = I2C_Read(BMP180_ADDRESS, BMP180_CTRL_MEAS);
  regData &= 0xC0;
  regData |= 0x34;
  I2C_Write(BMP180_ADDRESS, BMP180_CTRL_MEAS, regData);
  // Wait for measurement result available
  while ((I2C_Read(BMP180_ADDRESS, BMP180_CTRL_MEAS) & 0x20) != 0);

  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(BMP180_OUT_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP180_ADDRESS, 3);
  while (Wire.available() < 3);

  // Compute the raw pressure
  pressure = (long(Wire.read()) << 16) | (long(Wire.read()) << 8) | Wire.read();
  pressure >>= (8 - OSS);

  return (pressure);
}

// ************************************************************
// Function BMP180_readRawTemperature()
// Action : Read raw temperature registers
// Return value : Raw temperature measurement
long BMP180_readRawTemperature()
{
  long temperature;
  byte regData;

  // Start a pressure measurement
  regData = I2C_Read(BMP180_ADDRESS, BMP180_CTRL_MEAS);
  regData &= 0xC0;
  regData |= 0x2E;
  I2C_Write(BMP180_ADDRESS, BMP180_CTRL_MEAS, regData);
  // Wait for measurement result available
  while ((I2C_Read(BMP180_ADDRESS, BMP180_CTRL_MEAS) & 0x20) != 0);

  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(BMP180_OUT_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP180_ADDRESS, 2);
  while (Wire.available() < 2);

  // Compute the raw temperature
  temperature = (long(Wire.read()) << 8) | Wire.read();

  return (temperature);
}

void BMP180_getMeasurements(float &Temperature, float &Pressure, float &Altitude)
{
  // Acquire uncompsated temperature and pressure
  long UT = BMP180_readRawTemperature();
  long UP = BMP180_readRawPressure();

  // Temperature computation
  long X1 = (UT - AC6) * AC5 >> 15;
  long X2 = (long(MC) << 11) / (X1 + MD);
  long BB5 = X1 + X2;
  long T = (BB5 + 8) >> 4;

  // Pressure computation
  long BB6 = BB5 - 4000;
  X1 = (BB2 * ((BB6 * BB6) >> 12)) >> 11;
  X2 = (AC2 * BB6) >> 11;
  long X3 = X1 + X2;
  long BB3 = (((long(AC1) * 4 + X3) << OSS) + 2) >> 2;
  X1 = (AC3 * BB6) >> 13;
  X2 = (BB1 * ((BB6 * BB6) >> 12)) >> 16;
  X3 = (X1 + X2 + 2) >> 2;
  unsigned long BB4 = (AC4 * (unsigned long)(X3 + 32768)) >> 15;
  unsigned long BB7 = ((unsigned long)UP - BB3) * (50000 >> OSS);
  long p;
  if (BB7 < 0x80000000)
  {
    p = (BB7 << 1) / BB4;
  }
  else
  {
    p = (BB7 / BB4) << 1;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p = p + ((X1 + X2 + 3791) >> 4);

  Temperature = float(T) / 10;
  Pressure = float(p);

  // Compute altitude with linear barometric atmospheric model
  Altitude = (1 - Pressure / 101325) * 8434.516;
}

