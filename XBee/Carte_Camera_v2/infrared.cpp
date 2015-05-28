#include "infrared.h"
#include <SPI.h>

// Define /SS SPI pin for programmable amplifiers
// Amplifier 1 (IC4) : SSPIN = 0
// Amplifier 2 (IC5) : SSPIN = 1
#define SSPIN  10

// **********************************************************************
// INFRARED INTERFACE INITIALIZATION
// **********************************************************************
// ************************************************************
// Function Init_Infrared()
// Actions :
//    Initialize SPI interface for amplifiers
//    Set the maximum gain for both amplifiers
void Init_Infrared(void)
{
  // Set SSPIN as digital output
  pinMode(SSPIN, OUTPUT);
  // Start SPI interface and set SPI mode 0
  // Default SPI clock frequency is 4 MHz.
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);

  // Configure Amplifier 1 gain = 1
  PGA113_GainWrite(1, 0);

  // Configure Amplifier 2 gain = 1
  PGA113_GainWrite(2, 0);
}
// ************************************************************

// **********************************************************************
// PHOTOTRANSISTOR SELECTION FUNCTION
// **********************************************************************
// ************************************************************
// Function Select_InfraInput()
// Action : Select the infrared receiver cell
// Argument : Input number (from 1 to 10)
void Select_InfraInput(unsigned char num)
{
  // Set A0, A1, A2, A3 as digital inputs
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  switch (num)
  {
    case 1 :
      digitalWrite(A0, LOW);
      digitalWrite(A1, LOW);
      digitalWrite(A2, LOW);
      digitalWrite(A3, LOW);
      break;
    case 2 :
      digitalWrite(A0, HIGH);
      digitalWrite(A1, LOW);
      digitalWrite(A2, LOW);
      digitalWrite(A3, LOW);
      break;
    case 3 :
      digitalWrite(A0, LOW);
      digitalWrite(A1, HIGH);
      digitalWrite(A2, LOW);
      digitalWrite(A3, LOW);
      break;
    case 4 :
      digitalWrite(A0, HIGH);
      digitalWrite(A1, HIGH);
      digitalWrite(A2, LOW);
      digitalWrite(A3, LOW);
      break;
    case 5 :
      digitalWrite(A0, LOW);
      digitalWrite(A1, LOW);
      digitalWrite(A2, HIGH);
      digitalWrite(A3, LOW);
      break;
    case 6 :
      digitalWrite(A0, HIGH);
      digitalWrite(A1, LOW);
      digitalWrite(A2, HIGH);
      digitalWrite(A3, LOW);
      break;
    case 7 :
      digitalWrite(A0, LOW);
      digitalWrite(A1, HIGH);
      digitalWrite(A2, HIGH);
      digitalWrite(A3, LOW);
      break;
    case 8 :
      digitalWrite(A0, HIGH);
      digitalWrite(A1, HIGH);
      digitalWrite(A2, HIGH);
      digitalWrite(A3, LOW);
      break;
    case 9 :
      digitalWrite(A0, LOW);
      digitalWrite(A1, LOW);
      digitalWrite(A2, LOW);
      digitalWrite(A3, HIGH);
      break;
    case 10 :
      digitalWrite(A0, HIGH);
      digitalWrite(A1, LOW);
      digitalWrite(A2, LOW);
      digitalWrite(A3, HIGH);
      break;
  }
}
// ************************************************************

// **********************************************************************
// PGA113 FUNCTION
// **********************************************************************
// ************************************************************
// Function PGA113_GainWrite()
// Action : Write gain value in PGA113
// Arguments :
//        Gain code :
//  Gain Code    Amplification
//    0              1
//    1              2
//    2              5
//    3              10
//    4              20
//    5              50
//    6              100
//    7              200
//        numAmpli
// Amplifier number :
//  numAmpli = 1 : Amplifier 1 (IC4)
//  numAmpli = 2 : Amplifier 2 (IC5)
unsigned char PGA113_GainWrite(unsigned char numAmpli, unsigned char gain)
{
  // Set SSPIN active
  if (numAmpli == 1)
  {
    digitalWrite(SSPIN, HIGH);
    digitalWrite(SSPIN, LOW);
  }
  else
  {
    digitalWrite(SSPIN, LOW);
    digitalWrite(SSPIN, HIGH);
  }
  // Send Write command
  SPI.transfer(0x2A);
  // Send data (Channel = 0)
  SPI.transfer((gain << 4) & 0xF0);
  // Set SSPIN inactive
  if (numAmpli == 1)
  {
    digitalWrite(SSPIN, HIGH);
  }
  else
  {
    digitalWrite(SSPIN, LOW);
  }
  // Define amplification value (return value)
  switch (gain)
  {
    case 0 :  return (1);
    case 1 :  return (2);
    case 2 :  return (5);
    case 3 :  return (10);
    case 4 :  return (20);
    case 5 :  return (50);
    case 6 :  return (100);
    case 7 :  return (200);
  }
}
// ************************************************************

