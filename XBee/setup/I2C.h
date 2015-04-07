#include <Arduino.h>
#include <Wire.h>

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
byte I2C_Read(byte i2cAddr, byte regAddr);

// ************************************************************
// Function I2C_Write
// Action : Writes a byte in an I2C device
// Arguments :
//      i2cAddr  : I2C address of the device
//      regAddr  : Address of the register to be written
//      data     : Value to be written
void I2C_Write(byte i2cAddr, byte regAddr, byte data);
// ************************************************************
