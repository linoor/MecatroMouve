#include <stdlib.h>
#include <Wire.h>
#include <math.h>
#include "SoftwareSerial.h"
#include "Adafruit_GPS.h"
#include "elapsedMillis.h"
#include "../setup/I2C.cpp"
#include "../setup/BMP180.cpp"
#include "../setup/kalman.cpp"
#include "../setup/def.h"
#include "../setup/setup.cpp"

#define DEBUG
#define GPSECHO  false

float mTemperature, mPressure, mAltitude;
elapsedMillis timeElapsedAlt, timeElapsedLocation;
long refreshAlt, refreshLocation;


void senderConnect() {
    Serial.println("Setup started!");
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
                Serial.println("\nSetup finished!");
                return;
            }
        }
    }
}


int initCount = 0;
int count = 0;
bool clockwise = false;

void getTestLocation(int32_t* loc) {
    if (initCount > 20)
    {
        count += ((clockwise)? -5 : 5);
        // receivedLocation[0] = int32_t((48.0000 + 0.001 * count) * 10000000);
        loc[0] = int32_t((48.0000 + 0.0001 * cos(count*PI/180)) * 10000000);
        loc[1] = int32_t((2.0000 + 0.0001 * sin(count*PI/180)) * 10000000);
        // Serial.print(receivedLocation[0]);Serial.print(" ");Serial.println(receivedLocation[1]);
    }
    else {
        initCount++;
    }

    if (count > 600)
    {
        clockwise = true;
    }
    else if (count < -600)
    {
        clockwise = false;
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
    // getTestLocation(myLocation);
    bytes<int32_t> loc[2];
    loc[0].f = (int32_t)myLocation[0];
    loc[1].f = (int32_t)myLocation[1];
    sendData<int32_t>(loc, 2, "g");
    #ifdef DEBUG
    printDebugData<String>("\nCurrent Location: ");
    printDebugData<int32_t>(myLocation[0]);
    printDebugData<int32_t>(myLocation[1]);
    #endif
}

////////////  test sensors ////////////////

void testSensors()
{
    BMP180_getMeasurements(mTemperature, mPressure, mAltitude);
    float alti = mAltitude;
    printDebugData<String>("\nCurrent Altitude: ");
    printDebugData<float>(alti);

    int32_t myLocation[2];
    getGPSLocation(myLocation);
    printDebugData<String>("\nCurrent Location: ");
    printDebugData<int32_t>(myLocation[0]);
    printDebugData<int32_t>(myLocation[1]);
}

// use this ONLY when Xbee is not connected
void testSensors_()
{
    BMP180_getMeasurements(mTemperature, mPressure, mAltitude);
    float alti = mAltitude;
    Serial.println("\nCurrent Altitude: ");
    Serial.println(alti);

    int32_t myLocation[2];
    getGPSLocation(myLocation);
    Serial.println("\nCurrent Location: ");
    Serial.print(myLocation[0]);Serial.print(", ");
    Serial.println(myLocation[1]);
}

void setup()
{
    Serial.begin(57600);
    Wire.begin();
    setupBaro();
    setupGPS();

    flush();
    // senderConnect();

    flush();
    delay(500);
    flush();

    timeElapsedAlt = 0;
    timeElapsedLocation = 0;
    refreshAlt = 300;
    refreshLocation = 100;
}

void loop()
{
    if(timeElapsedAlt > refreshAlt){
        sendAltitude();
        timeElapsedAlt = 0;
    }
    // delay(300);
    if(timeElapsedLocation > refreshLocation){
        sendLocation();
    }
}
