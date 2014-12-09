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
// Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);


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
  Serial.println("Acc Start");
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
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
  float accGyro[6];
  getAccAndMag(accGyro);
  float pos[2];
  getGPSPosition(pos);

  for (int i = 0; i < 6; i++) {
    Serial.print(accGyro[i]);
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

void getAccAndMag(float* accGyro)
{
  sensors_event_t eventAcc;
  sensors_event_t eventGyro;

  accel.getEvent(&eventAcc);
  gyro.getEvent(&eventGyro);

  accGyro[0] = eventAcc.acceleration.x;
  accGyro[1] = eventAcc.acceleration.y;
  accGyro[2] = eventAcc.acceleration.z;
  accGyro[3] = eventGyro.gyro.x;
  accGyro[4] = eventGyro.gyro.y;
  accGyro[5] = eventGyro.gyro.z;
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



