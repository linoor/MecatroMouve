#include <stdlib.h>
#include <Wire.h>
#include <math.h>
#include "MPL3115A2.h"
#include "SoftwareSerial.h"
#include "TinyGPS++.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>

#define START_SIGNAL "s"
#define END_SIGNAL "e"
#define FLOAT_SIZE sizeof(float)

#define GPSRXPin 10
#define GPSTXPin 11
#define GPSBaud 9600

MPL3115A2 myPressure;

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

const int send_size = 9;

///////////////////////////////////////
//Les données
typedef byte uint8_t;

union float_bytes
{
  float f;
  byte b[FLOAT_SIZE];
};

float_bytes data[send_size];
float gpsPosition[2];
float accMagGyro[9];

///////////////////////////////////////
//mettre à jour les donnée

void updateData() {
   data[0].f = myPressure.readAltitude();

   getGPSPosition(gpsPosition);
   getAccAndMag(accMagGyro);

   data[1].f = gpsPosition[0];
   data[2].f = gpsPosition[1];
   data[3].f = accMagGyro[0];
   data[4].f = accMagGyro[1];
   data[5].f = accMagGyro[2];
   data[6].f = accMagGyro[3];
   data[7].f = accMagGyro[4];
   data[8].f = accMagGyro[5];
}

///////////////////////////////////////
//envoyer les données
void sendData() {
   //signaler le début
   Serial.print(START_SIGNAL);
   
   //envoyer de la merde
   for(int i = 0; i < send_size; i++){
       Serial.write(data[i].b, FLOAT_SIZE);
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

SoftwareSerial ss(GPSRXPin, GPSTXPin);
TinyGPSPlus gps;

////////////////////////////////////////
void setup()  
{
  Wire.begin();
  myPressure.begin();
  
  myPressure.setModeAltimeter();
  myPressure.setOversampleRate(7); // Pour lire 1 seule valeur, il lui faut 512ms
                                   // Du coup pas besoin de moyenner quoique ce soit!
  myPressure.enableEventFlags();

  /* Initialise the sensor */
  if(!accel.begin())
  {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  Serial.println("Acc Start");

  if(!gyro.begin())
  {
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println("Gyro Start");

  ss.begin(GPSBaud);
  Serial.println("GPS Start");
  
  Serial.begin(9600);
  flush();
  Serial.println("Setup started!");
  
  while(true) // żółć
  {
    Serial.print("A");
    delay(50);
    if(Serial.available()) {
      if(Serial.read() == 'B') {
        Serial.println("Setup finished!");
        break;
      }
    }
  }
  
  flush();
  delay(500);
  flush();
}

void loop()
{
  updateData();
  sendData();
  delay(5000);
}

void flush() {
  while(Serial.available()) {
    Serial.read();
  }
}

void getGPSPosition(float* pos)
{
  #ifdef DEBUG
  if (ss.available() <= 0)
  {
    Serial.println("GPS data not available...");
  }
  #endif

  while (ss.available() > 0) // As each character arrives...
  {
    char t = ss.read();
    gps.encode(t);
  }

  // if (gps.location.isUpdated() || gps.altitude.isUpdated()) {
  if (gps.location.isValid())
  {
    pos[0] = gps.location.lat();
    pos[1] = gps.location.lng();
  }
}

void getAccAndMag(float* accMagGyro)
{
  sensors_event_t eventAcc;
  sensors_event_t eventMag;
  sensors_event_t eventGyro;

  accel.getEvent(&eventAcc);
  mag.getEvent(&eventMag);
  gyro.getEvent(&eventGyro);

  accMagGyro[0] = eventAcc.acceleration.x;
  accMagGyro[1] = eventAcc.acceleration.y;
  accMagGyro[2] = eventAcc.acceleration.z;
  accMagGyro[3] = eventMag.magnetic.x;
  accMagGyro[4] = eventMag.magnetic.y;
  accMagGyro[5] = eventMag.magnetic.z;
  accMagGyro[6] = eventGyro.gyro.x;
  accMagGyro[7] = eventGyro.gyro.y;
  accMagGyro[8] = eventGyro.gyro.z;
}