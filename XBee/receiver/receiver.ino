#include <stdlib.h>
#include <Wire.h>
#include <math.h>
#include "SoftwareSerial.h"
#include "Adafruit_GPS.h"

#include "../setup/I2C.cpp"
#include "../setup/BMP180.cpp"
#include "../setup/kalman.cpp"
#include "../setup/def.h"
#include "../setup/setup.cpp"
#include "../setup/bearing.cpp"
#include "../moteur/Alexmos.h"

#define address 0x1E //0011110b, I2C 7bit address of HMC5883
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

float receivedAlti;
int32_t receivedLocation[2];

double bearing0, bearing1;
double verti0, verti1;

double bearingAngle, verticalAngle;


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
                    Serial.println("B");
                }
                return;
            }
        }
    }
}

// update data of receiver itself
void updateMyData()
{
    Serial.print("Updating my data... \nMy Altitude: ");
    BMP180_getMeasurements(mTemperature, mPressure, mAltitude);
    Serial.println(mAltitude);

    getGPSLocation(myLocation);
    Serial.print("My location: ");
    Serial.print(myLocation[0]);
    Serial.print(" ");
    Serial.println(myLocation[1]);
    Serial.println("Finished updating my data");
}

/// Update angles the motors should turn ///

void updateBearing()
{
    if (!isBearingInit)
    {
        bearing0 = computeBearing(receivedLocation, myLocation);
        isBearingInit = true;
        Serial.print("Bearing initialized");
    }
    else
    {
        bearing1 = computeBearing(receivedLocation, myLocation);
        Serial.print("Delta Bearing Angle: ");
        Serial.println(bearing1 - bearing0);
        bearingAngle = bearing1 - bearing0;
        bearing0 = bearing1;
    }
}

void updateVertical()
{
    if (!isVertiInit)
    {
        Serial.println("Initializing vertical");
        verti0 = computeVertical(receivedLocation, myLocation, receivedAlti, myAlti);
        isVertiInit = true;
        Serial.print("Vertical initialized");
    }
    else
    {
        verti1 = computeVertical(receivedLocation, myLocation, receivedAlti, myAlti);
        Serial.print("Delta Vertical Bearing Angle: ");
        Serial.println(verti1 - verti0);
        verticalAngle = verti1 - verti0;
        verti0 = verti1;
    }
}

////////// Receiving sender data ///////////

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
    Serial.println("Reading incoming data...");
    if (!Serial.available()) return;

    while (Serial.read() != START_SIGNAL)
    {
    }
    // delay(100);

    char type;
    float alti;
    int32_t gpsData[2];
    char r;
    r=Serial.read();
    Serial.println("Read data: ");
    Serial.println(r);

    switch (r)
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
    case 'd': // debug mode
        while(Serial.read() != END_SIGNAL) {}
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
            break;
        default:

            break;
        }
        updateMyData();
        printReceivedData();
        updateBearing();
        updateVertical();
    }
    Serial.println("Finished");
}

void sendMoteurCommand() {
    // testing
    camera_Roll += 1;
}

////////////////////////////////////////

void setup()
{
    Serial.begin(57600);
    Wire.begin();
    Serial.println("Setting up...");
    setupBaro();
    setupGPS();

    flush();
    receiverConnect();

    flush();
    delay(500);
}

void loop()
{
    receiveData();
    sendMoteurCommand();
    delay(200);
}
