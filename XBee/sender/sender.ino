#include <stdlib.h>
#include <Wire.h>
#include <math.h>
#include "MPL3115A2.h"
#include "SoftwareSerial.h"
#include "TinyGPS++.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>

#include "../setup/def.h"
#include "../setup/setup.cpp"

#define START_SIGNAL "s"
#define END_SIGNAL "e"
#define SEND_SIZE 1
#define FLOAT_SIZE sizeof(float)

#define GPSRXPin 4
#define GPSTXPin 5
// #define GPSBaud 9600

// #define DEBUG

MPL3115A2 myPressure;

SoftwareSerial ss(GPSRXPin, GPSTXPin);
TinyGPSPlus gps;

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

bytes<float> data[SEND_SIZE];

////////// prototype //////////////////
// void getGPSPosition(SoftwareSerial ss, float *pos);

////////// setting things up //////////

void setupAccMagGyro()
{
    /* Initialise the sensor */
    if (!accel.begin() || !mag.begin())
    {
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while (1);
    }
    Serial.println("Acc + Mag started...");

    if (!gyro.begin())
    {
        Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    Serial.println("Gyro started...");
}

void senderConnect() {
    while (true)
    {
        Serial.print("A");
        delay(50);

        while (Serial.available())
        {
            // Serial.println("Available");
            if (Serial.read() == 'B')
            {
                // Serial.println("Read B");
                Serial.println("Setup finished!");
                return;
            }
        }
    }
}

////////// sending data //////////

///////////////////////////////////////
//mettre à jour les donnée
/*void updateData()
{
    float gpsPosition[2];
    float accMagGyro[9];

    getGPSPosition(gpsPosition);
    // getAccMagGyro(accMagGyro);

    data[0].f = myPressure.readAltitude();
    data[1].f = gpsPosition[0]; // Latitude
    data[2].f = gpsPosition[1]; // Longitude
    data[3].f = 0;  // Acc x
    data[4].f = 0;  // Acc y
    data[5].f = 0;  // Acc z
    data[6].f = 0;  // Gyro x
    data[7].f = 0;  // Gyro y
    data[8].f = 0;  // Gyro z

    // Serial.println(myPressure.readAltitude());
    // Serial.println(gpsPosition[0]);
    // Serial.println(gpsPosition[1]);

// #ifdef DEBUG
    // Serial.println();
    // for (int i = 0; i < SEND_SIZE; i++) {
    //     Serial.print(data[i].f);
    // }
    // Serial.println();
// #endif
    // delay(100);
}*/

void getAccMagGyro(float *accMagGyro)
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

//////////////////////////////////////////////
// Debugging phase

int8_t counter = 0;

template <typename T>
void sendData(bytes<T> dataToSend[], int dataSize, String typeSignal)
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

template <typename T>
void printDebugData(T debugData, String debugMsg)
{
    Serial.print(START_SIGNAL);
    Serial.print("d");
    Serial.println(debugMsg);
    Serial.println(debugData);
    Serial.print(END_SIGNAL);
}

void sendAltitude()
{
    bytes<float> alti[1];
    alti[0].f = myPressure.readAltitude();
    sendData<float>(alti, 1, "a");
    #if DEBUG
    printDebugData<float>(alti[0].f);
    #endif
}

void sendLocation()
{
    long loc[2];
    getGPSPosition(ss, gps, loc);

    bytes<long> locByte[2];
    locByte[0].f = loc[0];
    locByte[1].f = loc[1];
    sendData<long>(locByte, 2, "g");

    #if DEBUG
    printDebugData<long>(loc[0], "Latitude: ");
    printDebugData<long>(loc[1], "Longitude: ");
    #endif
}

//////////////////////////////////////////////
// test XBee

float counterF = 0.0;

void sendFloatTest()
{
    bytes<float> data[1];
    data[0].f = counterF += 0.5;
    sendData<float>(data, 1, "a");
}

long counterL = 0;
void sendLongTest()
{
    bytes<long> data[1];
    data[0].f = counterL++;
    sendData<long>(data, 1, "l");
}

int counterI = 0;
void sendIntTest()
{
    bytes<int> data[1];
    data[0].f = counterI++;
    sendData<int>(data, 1, "i");
}

////////////////////////////////////////
// test sensors
void testGPS()
{
    long gpsPosition[2];
    getGPSPosition(ss, gps, gpsPosition);
    Serial.print("GPS: ");
    Serial.print(gpsPosition[0]);
    Serial.print(", ");
    Serial.println(gpsPosition[1]);
}

void testBaro()
{
    Serial.print("Altitude: ");
    Serial.println(myPressure.readAltitude());
}


void setup()
{
    Wire.begin();

    setupBaro(myPressure);
    setupGPS(ss);
    // setupAccMagGyro();

    Serial.begin(57600);
    flush();
    Serial.println("Setup started!");

    // senderConnect();

    flush();
    delay(500);
    flush();
}

void loop()
{
    // sendTest();
    // updateData();
    // sendFloatTest();
    // sendLongTest();
    // sendIntTest();
    // sendAltitude();
    testBaro();
    testGPS();
    delay(300);
}
