#include <stdlib.h>
#include <Wire.h>
#include <Servo.h>
#include "MPL3115A2.h"
#include <math.h>
#include "global.h"

#define DISTANCE 3 // en mètres
#define FLOAT_SIZE sizeof(float)
#define START_SIGNAL 's'
#define END_SIGNAL 'e'
#define RECEIVE_SIZE 4
#define LOCAL_SIZE 12

////////////////////////////////////////
//les données de CET Arduino
float data[LOCAL_SIZE];

MPL3115A2 myPressure;
Servo myservoVertical, myservoHorizontal;
float receive_pressure, mesure_pressure, diff_pressure;

////////////////////////////////////////
//mettre à jour les donnée de CET arduino
void updateData(){
}

////////////////////////////////////////
//recevoir les données
void readData(){
   if(!Serial.available())
      return;
   else{
      if(Serial.read() != START_SIGNAL)
          return;
      else{
          float temp[RECEIVE_SIZE];
          for(int i = 0 ; i< RECEIVE_SIZE ; i++){
              temp[i] = readFloat();
          }
          if(Serial.read() != END_SIGNAL)
              return;
          else for(int i = 0 ; i< RECEIVE_SIZE ; i++){
              dataReceived[i] = temp[i];
              Serial.print("I fucking received some shit! it is : ");
              Serial.println(dataReceived[i]);
          }
      }
   }
}

float readFloat()
{
  float_bytes payload;
  
  while(!Serial.available());
 
  for(int i = 0; i < FLOAT_SIZE; i++)
  {
    payload.b[i] = Serial.read();
    //delay(5);
  }
  
  return payload.f;
}
////////////////////////////////////////
//bouge les servos!!!!!
void moveCamera(){

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
  
  myservoVertical.attach(10); // Attach servos
  myservoHorizontal.attach(11);
  myservoVertical.write(90);
  myservoHorizontal.write(0);
  
  Serial.begin(9600);
  while(Serial.available()) // flush
    Serial.read();
  Serial.println("Debut! And looking for A!");
  
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
  readData();
  updateData();
  moveCamera();
  
  
  /*
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
  */
  
  delay(100);
}

int parse_MinMax(int val, int mini, int maxi)
{
  return (val > maxi) ? maxi : (val < mini) ? mini : val;
}


