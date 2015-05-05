#include <stdlib.h>
#include <Wire.h>
#include <math.h>
#include "SoftwareSerial.h"
#include "MPL3115A2.h"
// #include "TinyGPS++.h"
#include "Adafruit_GPS.h"

#include "../setup/I2C.cpp"
#include "../setup/BMP180.cpp"
#include "../setup/kalman.cpp"
#include "../setup/def.h"
#include "../setup/setup.cpp"

#define SEND_SIZE 1

#define DEBUG
#define GPSECHO  false

float mTemperature, mPressure, mAltitude;


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
void printDebugData(T debugData)
{
    Serial.print(START_SIGNAL);
    Serial.print("d");
    Serial.println(debugData);
    Serial.println();
    Serial.print(END_SIGNAL);
}

void sendAltitude()
{
    bytes<float> alti[1];
    BMP180_getMeasurements(mTemperature, mPressure, mAltitude);
    alti[0].f = mAltitude;
    sendData<float>(alti, 1, "a");
    #ifdef DEBUG
    printDebugData<String>("\nCurrent Altitude: ");
    printDebugData<float>(alti[0].f);
    #endif
}

void sendLocation()
{
    int32_t myLocation[2];
    getGPSLocation(myLocation);
    bytes<int32_t> loc[2];
    loc[0].f = (int32_t)myLocation[0];
    loc[1].f = (int32_t)myLocation[1];
    sendData<int32_t>(loc, 2, "g");
    #ifdef DEBUG
    printDebugData<String>("Current Location: ");
    printDebugData<int32_t>(myLocation[0]);
    printDebugData<int32_t>(myLocation[1]);
    #endif
}

////////////  test sensors ////////////////

void testSensors()
{
    BMP180_getMeasurements(mTemperature, mPressure, mAltitude);
    float alti = mAltitude;
    printDebugData<String>("Current Altitude: ");
    printDebugData<float>(alti);

    int32_t myLocation[2];
    getGPSLocation(myLocation);
    printDebugData<String>("Current Location: ");
    printDebugData<int32_t>(myLocation[0]);
    printDebugData<int32_t>(myLocation[1]);
}


void setup()
{
    Serial.begin(57600);

    Wire.begin();
    setupBaro();
    setupGPS();

    flush();
    Serial.println("Setup started!");

    senderConnect();

    flush();
    delay(500);
    flush();
}

void loop()
{
    // sendAltitude();
    // delay(300);
    sendLocation();
    // testSensors();
    delay(300);
}
