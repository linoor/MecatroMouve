#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include "SoftwareSerial.h"
#include "TinyGPS++.h"

#define RXPin 52
#define TXPin 53
#define GPSBaud 9600
#define ConsoleBaud 115200

// #define DEBUG

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
TinyGPSPlus gps;

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);


void setup()
{
  Serial.begin(ConsoleBaud);

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

  delay(500);
}

void loop()
{
  float accMagGyro[9];
  getAccAndMag(accMagGyro);
  float pos[2];
  getGPSPosition(pos);

  for (int i = 0; i < 9; i++) {
    Serial.print(accMagGyro[i]);
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
