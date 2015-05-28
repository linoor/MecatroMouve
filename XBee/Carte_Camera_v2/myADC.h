#include <Arduino.h>

// **********************************************************************
// ANALOG TO DIGITAL FUNCTIONS
// **********************************************************************
// ************************************************************
// Function ADC_init
// Action : Initialize ADC converter
// Argument : ADC channel (from 0 to 7)
void ADC_init(unsigned char channel);

// ************************************************************
// Function ADC_get
// Action : Read AD conversion result
// Argument : ADC channel (from 0 to 7)
// Return value : ADC result (from 0 to 1023)
unsigned int ADC_get(unsigned char channel);
