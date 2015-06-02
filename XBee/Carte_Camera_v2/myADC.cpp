#include "myADC.h"

// **********************************************************************
// ANALOG TO DIGITAL FUNCTIONS
// **********************************************************************
// ************************************************************
// Function ADC_init
// Action : Initialize ADC converter
// Argument : ADC channel (from 0 to 7)
void ADC_init(unsigned char channel)
{
  ADMUX = 0x40 + channel;
  ADCSRA = 0x82;
  ADCSRB &= 0xF7;
}

// ************************************************************
// Function ADC_get
// Action : Read AD conversion result
// Argument : ADC channel (from 0 to 7)
// Return value : ADC result (from 0 to 1023)
unsigned int ADC_get(unsigned char channel)
{
  int result;

  // Start conversion
  ADCSRA |= 0x40;

  // Wait for conversion completion
  while (ADCSRA & 0x40);

  // ADCL must be read first
  result = ADCL;
  result = result | ((ADCH << 8) & 0xFF00);

  return (result);
}
