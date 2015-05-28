#include <Arduino.h>

// **********************************************************************
// INFRARED INTERFACE INITIALIZATION
// **********************************************************************
// ************************************************************
// Function Init_Infrared()
// Actions :
//    Initialize SPI interface for amplifiers
//    Set the maximum gain for both amplifiers
void Init_Infrared(void);

// **********************************************************************
// PHOTOTRANSISTOR SELECTION FUNCTION
// **********************************************************************
// ************************************************************
// Function Select_InfraInput()
// Action : Select the infrared receiver cell
// Argument : Input number (from 1 to10)
void Select_InfraInput(unsigned char num);

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
//        numAmpli :
// Amplifier number :
//  numAmpli = 1 : Amplifier 1 (IC4)
//  numAmpli = 2 : Amplifier 2 (IC5)
// Return value :
//    Current amplifier gain value
unsigned char PGA113_GainWrite(unsigned char numAmpli, unsigned char gain);
