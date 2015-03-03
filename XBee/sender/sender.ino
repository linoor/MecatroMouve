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
#define SEND_SIZE 9
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

union float_bytes
{
    float f;
    byte b[FLOAT_SIZE];
};

float_bytes data[SEND_SIZE];

////////// setting things up //////////

void setupBaro()
{
    myPressure.begin();

    myPressure.setModeAltimeter();
    myPressure.setOversampleRate(7); // Pour lire 1 seule valeur, il lui faut 512ms
    // Du coup pas besoin de moyenner quoique ce soit!
    myPressure.enableEventFlags();
}

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

void setupGPS()
{
    ss.begin(GPSBaud);
    Serial.println("GPS started...");
}

void testConnection() {
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

void flush()
{
    while (Serial.available())
    {
        Serial.read();
    }
}

////////// sending data //////////

///////////////////////////////////////
//mettre à jour les donnée
void updateData()
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
}

///////////////////////////////////////
//envoyer les données
void sendData()
{
    //signaler le début
    Serial.print(START_SIGNAL);

    //envoyer de la merde
    for (int i = 0; i < SEND_SIZE; i++)
    {
        Serial.write(data[i].b, FLOAT_SIZE);
    }

    //signaler la fin
    Serial.print(END_SIGNAL);
}

void getGPSPosition(float *pos)
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

int8_t counter = 0;

void sendTest() {
    if (counter > 256) return;
    Serial.write(counter++);
    // float_bytes testData;
    // testData.f = counter++;
    // Serial.write(testData.b, FLOAT_SIZE);
}

////////////////////////////////////////

void setup()
{
    Wire.begin();

    setupBaro();
    // setupGPS();
    // setupAccMagGyro();

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
    // sendTest();
    // updateData();
    // sendData();
    delay(300);
}
