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
#define SEND_SIZE 1
#define FLOAT_SIZE sizeof(float)

#define GPSRXPin 10
#define GPSTXPin 11
#define GPSBaud 9600

// #define DEBUG

MPL3115A2 myPressure;

SoftwareSerial ss(GPSRXPin, GPSTXPin);
TinyGPSPlus gps;

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

///////////////////////////////////////
//Les données
typedef byte uint8_t;

template <typename T>
union bytes
{
    T f;
    byte b[sizeof(T)];
};

//////////////////////////////////////////////
// Debugging phase
float counterF = 0.0;

template <typename T>
void sendTestData(bytes<T> dataToSend[], int dataSize, String typeSignal)
{
    //signaler le début
    Serial.print(START_SIGNAL);
    Serial.print(typeSignal);
    //envoyer de la merde
    for (int i = 0; i < dataSize; i++)
    {
        Serial.write(dataToSend[i].b, sizeof(T));
    }
    //signaler la fin
    Serial.print(END_SIGNAL);
}

void sendFloatTest()
{
    bytes<float> data[1];
    data[0].f = counterF += 0.5;
    sendTestData<float>(data, 1, "a");
}

////////////////////////////////////////

void setup()
{
    Wire.begin();

    Serial.begin(57600);
    flush();
    Serial.println("Setup started!");

    testConnection();

    flush();
    delay(500);
    flush();
}

void loop()
{
    sendFloatTest();
    delay(300);
}
