#include <stdlib.h>
#include <Wire.h>
#include <Servo.h>
#include "MPL3115A2.h"
#include <math.h>
#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>

#define DISTANCE 3 // en mètres
#define FLOAT_SIZE sizeof(float)
#define START_SIGNAL 's'
#define END_SIGNAL 'e'
#define RECEIVE_SIZE 9
#define LOCAL_SIZE 12

#define GPSRXPin 8
#define GPSTXPin 9
#define GPSBaud 9600

#define R 6371000

template <typename T>
union bytes
{
    T f;
    byte b[sizeof(T)];
};

////////////////////////////////////////

template <typename T>
T readData() {
    bytes<T> received;
    for (int i = 0; i < sizeof(T); i++)
    {
        received.b[i] = Serial.read();
    }
    return received.f;
}

void readTest()
{
    if (!Serial.available()) return;

    int counter = 0;
    while (Serial.read() != START_SIGNAL)
    {
    }
    // delay(100);

    float alti;
    int32_t gpsPosition[2];

    switch (Serial.read())
    {
        case 'a':
            alti = readData<float>();
            break;
        default:
            break;
    }

    // delay(100);
    // Wait for END_SIGNAL
    if (Serial.read() != END_SIGNAL)
    {
        // Serial.println(alti);
        Serial.println("END_SIGNAL not received... Truncate data...");
    }
    else
    {
        // for (int i = 0 ; i < RECEIVE_SIZE ; i++)
        // {
        //     dataReceived[i] = temp[i];
        // }
        // printDataReceived();
        Serial.println(alti);
    }
    Serial.println();
}


////////////////////////////////////////

void setup()
{
    Wire.begin();

    Serial.begin(57600);
    flush();

    testConnection();

    flush();
    delay(500);
}

void loop() // run over and over
{
    readTest();
    delay(300);
}

