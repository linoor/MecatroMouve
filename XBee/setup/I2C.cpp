#include "I2C.h"

// **********************************************************************
// COMMUNICATION FUNCTIONS
// **********************************************************************
// ************************************************************
// Function I2C_Read
// Action : Reads a byte from an I2C device
// Arguments :
//      i2cAddr : I2C address of the device
//      regAddr : Address of the register to be read
// Return value : Register byte value
byte I2C_Read(uint8_t i2cAddr, uint8_t regAddr)
{
  Wire.beginTransmission(i2cAddr);
  Wire.write(regAddr);          // Send register's address
  Wire.endTransmission(false);  // Send repeated Start
  Wire.requestFrom(i2cAddr, (uint8_t)1);
  return Wire.read();
}
// ************************************************************
// Function I2C_Write
// Action : Writes a byte in an I2C device
// Arguments :
//      i2cAddr  : I2C address of the device
//      regAddr  : Address of the register to be written
//      data     : Value to be written
void I2C_Write(byte i2cAddr, byte regAddr, byte data)
{
  Wire.beginTransmission(i2cAddr);
  Wire.write(regAddr);         // Send register's address
  Wire.write(data);            // Send data to write
  Wire.endTransmission(true);  // End transmission
}
