#include <stdlib.h>
#include <Wire.h>
#include <Servo.h>
#include "MPL3115A2.h"
#include <math.h>

#define DISTANCE 3 // en mètres
#define FLOAT_SIZE sizeof(float)

union float_bytes
{
  float f;
  uint8_t b[FLOAT_SIZE];
};

MPL3115A2 myPressure;
Servo myservoVertical, myservoHorizontal;
float receive_pressure, mesure_pressure, diff_pressure;

float_bytes payload;

void setup()  
{
  Wire.begin();
  myPressure.begin();
  
  myPressure.setModeAltimeter();
  myPressure.setOversampleRate(7); // Pour lire 1 seule valeur, il lui faut 512ms
                                   // Du coup pas besoin de moyenner quoique ce soit!
  myPressure.enableEventFlags();
  
  myservoVertical.attach(10); // Attach servos
  myservoHorizontal.attach(11);
  myservoVertical.write(90);
  myservoHorizontal.write(0);
  
  Serial.begin(9600);
  while(Serial.available()) // flush
    Serial.read();
  Serial.println("Debut!");
  
  //char received;
  while(true)
  {
    delay(10);
    if(Serial.available())
    {
      if(Serial.read() == 'A')
      {
        for(int i = 0; i < 5; i++)
          Serial.print("B");
        break;
      }
    }
  }
  
  while(Serial.available()) // flush
    Serial.read();//*/
    
  delay(200);
  
}

void loop() // run over and over
{
  Serial.println("Recieved 1");
  receive_pressure = readFloat();
  Serial.print("recieve_pressure : ");
  Serial.println(receive_pressure);
  mesure_pressure = myPressure.readAltitude(0x60);
  Serial.print("mesure_pressure : ");
  Serial.println(mesure_pressure);
  diff_pressure = receive_pressure - mesure_pressure;
  Serial.print("diff_pressure : ");
  Serial.println(diff_pressure);
  
  myservoVertical.write(parse_MinMax(57.32*(1.57 - atan(diff_pressure/DISTANCE)), 10, 170));
  
  delay(30);
}

int parse_MinMax(int val, int mini, int maxi)
{
  return (val > maxi) ? maxi : (val < mini) ? mini : val;
}

float readFloat()
{
  float_bytes payload;
  
  while(!Serial.available());
 
  for(int i = 0; i < FLOAT_SIZE; i++)
  {
    payload.b[i] = Serial.read();
    delay(5);
  }
  
  return payload.f;
}
