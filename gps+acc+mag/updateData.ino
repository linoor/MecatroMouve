#include <Wire.h>
#include "SoftwareSerial.h"
#include "TinyGPS++.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#define RXPin 52
#define TXPin 53
#define GPSBaud 9600
#define ConsoleBaud 115200

// #define DEBUG

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
TinyGPSPlus gps;

/* Assign a unique ID to the sensors */
// Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);


void setup()
{
  Serial.begin(ConsoleBaud);

  /* Enable auto-gain (from example) */
  // mag.enableAutoRange(true);

  /* Initialise the sensor */
  if(!accel.begin() || !mag.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  Serial.println("Acc+Mag Start");

  ss.begin(GPSBaud);
  Serial.println("GPS Start");

  delay(500);
}

void loop()
{
  float accMag[6];
  getAccAndMag(accMag);
  float pos[2];
  getGPSPosition(pos);

  for (int i = 0; i < 6; i++) {
    Serial.print(accMag[i]);
    Serial.print(',');
  }
  Serial.println();

  for (int i = 0; i < 2; i++) {
    Serial.print(pos[i]);
    Serial.print(',');
  }
  Serial.println();

  delay(200);
}

void getAccAndMag(float* accMag)
{
  sensors_event_t eventAcc;
  sensors_event_t eventMag;

  accel.getEvent(&eventAcc);
  mag.getEvent(&eventMag);

  accMag[0] = (float)eventAcc.acceleration.x;
  accMag[1] = (float)eventAcc.acceleration.y;
  accMag[2] = (float)eventAcc.acceleration.z;
  accMag[3] = (float)eventMag.magnetic.x;
  accMag[4] = (float)eventMag.magnetic.y;
  accMag[5] = (float)eventMag.magnetic.z;
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



