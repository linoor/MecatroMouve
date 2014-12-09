#include <stdlib.h>
#include <Wire.h>
#include "MPL3115A2.h"
#include <math.h>

#define FLOAT_SIZE sizeof(float)
#define START_SIGNAL 's'
#define END_SIGNAL 'e'

MPL3115A2 myPressure;

///////////////////////////////////////
//Les données
union float_bytes
{
  float f;
  uint8_t b[FLOAT_SIZE];
};

float_bytes data[4];

///////////////////////////////////////
//mettre à jour les donnée
int time = 1;

void updateData(){
   data[0].f = myPressure.readAltitude(0x60);
   data[1].f = 1*time;
   data[2].f = 2*time;
   data[3].f = 3*time;
   //data[4].f = 4*time;
   //data[5].f = 5*time;
   //data[6].f = 6*time;
   //data[7].f = 7*time;
   //data[8].f = 8*time;
   time++;
}

///////////////////////////////////////
//envoyer les données
void sendData(){
   //signaler le début
   Serial.print(START_SIGNAL);
   
   //envoyer de la merde
   for(int i = 0 ; i< 10 ; i++){
       Serial.write(data[i].b , FLOAT_SIZE);
   }
   
   //signaler la fin
   Serial.print(END_SIGNAL);
}

void send_float(float f)
{
  float_bytes payload;
  payload.f = f;
  Serial.write(payload.b, FLOAT_SIZE);
}

////////////////////////////////////////
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
  updateData();
  sendData();
  delay(200);
}


