#include <stdlib.h>
#include <Wire.h>
#include "MPL3115A2.h"
#include <math.h>

#define FLOAT_SIZE sizeof(float)

union float_bytes
{
  float f;
  uint8_t b[FLOAT_SIZE];
};

MPL3115A2 myPressure;

void setup()  
{
  Wire.begin();
  myPressure.begin();
  
  myPressure.setModeAltimeter();
  myPressure.setOversampleRate(7); // Pour lire 1 seule valeur, il lui faut 512ms
                                   // Du coup pas besoin de moyenner quoique ce soit!
  myPressure.enableEventFlags();
  
  Serial.begin(9600);
  while(Serial.available()) // flush
    Serial.read();
  Serial.println("Debut!");
  
  while(true)
  {
    Serial.print("A");
    delay(50);
    if(Serial.available())
    {
      if(Serial.read() == 'B')
        break;
    }
  }
  
  while(Serial.available()) // flush
    Serial.read();
    
  delay(500);
  
  while(Serial.available()) // flush
    Serial.read();
}

void loop()
{
  send_float(myPressure.readAltitude(0x60));
  delay(1000);
}

void send_float(float f)
{
  float_bytes payload;
  payload.f = f;
  Serial.write(payload.b, FLOAT_SIZE);
}

