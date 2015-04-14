#include <stdlib.h>
#include <Wire.h>
#include <math.h>
#include "SoftwareSerial.h"
#include "MPL3115A2.h"
#include "Adafruit_GPS.h"

#include "../setup/I2C.cpp"
#include "../setup/BMP180.cpp"
#include "../setup/kalman.cpp"
#include "../setup/def.h"
#include "../setup/setup.cpp"
#include "../setup/bearing.cpp"

#define DISTANCE 3 // en mètres
#define IDLE_STEPS 80
#define CALIBRATION_STEPS 20

// #define DEBUG

/*
* Global constants
*/
float receive_pressure, mesure_pressure, diff_pressure;
float mTemperature, mPressure, mAltitude;
float correction = 0;
int counter = 0;

float myAlti;
// double myLocation[2];
int32_t myLocation[2];

double bearing0;
double bearing1;

float receivedAlti;
int32_t receivedLocation[2];


// establish connection with sender
void receiverConnect()
{
    Serial.println("Start looking for A!");
    while (true)
    {
        delay(10);
        if (Serial.available())
        {
            char rec = Serial.read();
            Serial.print(rec);
            if (rec == 'A')
            {
                for (int i = 0; i < 10; i++)
                {
                    Serial.print("B");
                }
                return;
            }
        }
    }
}

// update data of receiver itself
void updateMyData()
{
    Serial.print("Updating my data...");
    BMP180_getMeasurements(mTemperature, mPressure, mAltitude);
    Serial.println(mAltitude);

    getGPSLocation(myLocation);
    Serial.print(myLocation[0]);
    Serial.print(" ");
    Serial.println(myLocation[1]);
}

////////////////////////////////////////

void calibrateAltitude(float alti)
{
    Serial.println(counter);

    if (counter < IDLE_STEPS)
    {
        counter++;
    }
    else {
        if (counter - IDLE_STEPS < CALIBRATION_STEPS)
        {
            correction += (alti - myAlti);
            counter++;
        }
        else if (counter - IDLE_STEPS == CALIBRATION_STEPS)
        {
            correction = correction / CALIBRATION_STEPS;
            counter++;
            // Serial.println(correction);
        }
    }
}

////////// Receiving sender data //////////

// after parsing START_SIGNAL and TYPE_SIGNAL
// read datum of type @T (byte by byte), then return it
template <typename T>
T readData() {
    bytes<T> received;
    for (int i = 0; i < sizeof(T); i++)
    {
        received.b[i] = Serial.read();
    }
    return received.f;
}

// after parsing START_SIGNAL and TYPE_SIGNAL
// read data array of size @dataSize,
// then update the passed-in reference
template <typename T>
void readData(T* data, int dataSize)
{
    bytes<T> received[dataSize];
    while (!Serial.available());
    for (int i = 0; i < dataSize; i++)
    {
        for (int j = 0; j < sizeof(T); j++)
        {
            received[i].b[j] = Serial.read();
        }
        data[i] = received[i].f;
    }
}

void printReceivedData()
{
    Serial.print("Altitude received: ");
    Serial.println(receivedAlti);
    Serial.print("Location received: ");
    Serial.print(receivedLocation[0]);
    Serial.print(" ");
    Serial.println(receivedLocation[1]);
}

// receive data from sender
// parse and update data
void receiveData()
{
    if (!Serial.available()) return;

    while (Serial.read() != START_SIGNAL)
    {
    }
    // delay(100);

    char type;

    float alti;
    int32_t gpsData[2];
    long testLong;
    int testInt;

    switch (Serial.read())
    {
    case 'a':
        type = 'a';
        alti = readData<float>();
        break;
    case 'g':
        // receiving gps data
        type = 'g';
        readData<int32_t>(gpsData, 2);
        break;
    case 'l': // test for sending long
        Serial.println("l");
        testLong = readData<long>();
        break;
    case 'i':
        Serial.println("i");
        testInt = readData<int>();
        break;
    case 'd': // debug mode
        // Serial.println("Start looking for end signal");
        while(Serial.read() != END_SIGNAL)
        {
        }
        // Serial.println("Found end signal. Returning");
        return;
    default:
        break;
    }

    // Wait for END_SIGNAL
    if (Serial.read() != END_SIGNAL)
    {
        // Serial.println("END_SIGNAL not received... Truncate data...");
    }
    else
    {
        Serial.println(type);
        switch (type)
        {
        case 'a':
            receivedAlti = alti;
            break;
        case 'g':
            receivedLocation[0] = gpsData[0];
            receivedLocation[1] = gpsData[1];

            if (isInit)
            {
                bearing0 = computeBearing(receivedLocation, myLocation);
                isInit = false;
            }
            else
            {
                bearing1 = computeBearing(receivedLocation, myLocation);
                Serial.print("Delta Bearing Angle: ");
                Serial.println(bearing1 - bearing0);
                bearing0 = bearing1;
            }
            break;
        default:
            break;
        }
        printReceivedData();
    }
}

////////////////////////////////////////

void setup()
{
    Wire.begin();
    setupBaro();
    setupGPS();

    Serial.begin(57600);
    flush();

    //receiverConnect();

    flush();
    delay(500);
}

void loop()
{
    updateMyData();
    receiveData();
    delay(300);
}
