#include <stdlib.h>
#include <Wire.h>
#include <Servo.h>
#include "MPL3115A2.h"
#include <math.h>

#define DISTANCE 3 // en mètres
#define FLOAT_SIZE sizeof(float)
#define START_SIGNAL 's'
#define END_SIGNAL 'e'
#define RECEIVE_SIZE 9
#define LOCAL_SIZE 12

union float_bytes
{
    float f;
    uint8_t b[FLOAT_SIZE];
};

///////////////////////////////////////
//Les données reçus
float dataReceived[RECEIVE_SIZE];

////////////////////////////////////////
//les données de CET Arduino
float data[LOCAL_SIZE];

MPL3115A2 myPressure;
Servo myservoVertical, myservoHorizontal;
float receive_pressure, mesure_pressure, diff_pressure;


////////// setting things up //////////

void setupBaro()
{
    myPressure.begin();

    myPressure.setModeAltimeter();
    myPressure.setOversampleRate(7); // Pour lire 1 seule valeur, il lui faut 512ms
    // Du coup pas besoin de moyenner quoique ce soit!
    myPressure.enableEventFlags();
}

void setupServo()
{
    // myservoVertical.attach(10); // Attach servos
    // myservoHorizontal.attach(11);
    // myservoVertical.write(90);
    // myservoHorizontal.write(0);
}

void testConnection()
{
    Serial.println("Start looking for A!");
    while (true)
    {
        delay(10);
        if (Serial.available())
        {
            if (Serial.read() == 'A')

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

void flush()
{
    while (Serial.available())
    {
        Serial.read();
    }
}

////////// handling data //////////

////////////////////////////////////////
//mettre à jour les donnée de CET arduino
void updateData()
{
}

////////////////////////////////////////
//bouge les servos!!!!!
void moveCamera()
{
}

////////////////////////////////////////
//recevoir les données
void readData()
{
    if (!Serial.available())
    {
        return;
    }
    else
    {
        // Wait for START_SIGNAL
        while (Serial.read() != START_SIGNAL)
        {
            // Serial.println("START_SIGNAL not received...");
        }
        Serial.println("START_SIGNAL success!!");
        delay(100);

        // Receiving RECEIVE_SIZE floats
        float temp[RECEIVE_SIZE];
        for (int i = 0 ; i < RECEIVE_SIZE ; i++)
        {
            temp[i] = readFloat();
        }

        // Wait for END_SIGNAL
        if (Serial.read() != END_SIGNAL)
        {
            Serial.println("END_SIGNAL not received...");
            // Truncate data if END_SIGNAL not received
        }
        else
        {
            // Serial.println("END_SIGNAL success!!");
            for (int i = 0 ; i < RECEIVE_SIZE ; i++)
            {
                dataReceived[i] = temp[i];
            }
            printDataReceived();
        }
        Serial.println();
    }
}

float readFloat()
{
    float_bytes payload;

    while (!Serial.available());

    for (int i = 0; i < FLOAT_SIZE; i++)
    {
        payload.b[i] = Serial.read();
        // delay(5);
    }

    return payload.f;
}

void printDataReceived()
{
    Serial.print("Altitude (Baro) : "); Serial.println(dataReceived[0]);
    Serial.print("Longitude (GPS) : "); Serial.println(dataReceived[1]);
    Serial.print("Latitude  (GPS) : "); Serial.println(dataReceived[2]);
    Serial.print("Acc  x   (9DoG) : "); Serial.println(dataReceived[3]);
    Serial.print("Acc  y   (9DoG) : "); Serial.println(dataReceived[4]);
    Serial.print("Acc  z   (9DoG) : "); Serial.println(dataReceived[5]);
    Serial.print("Gyro x   (9DoG) : "); Serial.println(dataReceived[6]);
    Serial.print("Gyro y   (9DoG) : "); Serial.println(dataReceived[7]);
    Serial.print("Gyro z   (9DoG) : "); Serial.println(dataReceived[8]);
}

////////////////////////////////////////

void setup()
{
    Wire.begin();

    setupBaro();
    setupServo();

    Serial.begin(57600);
    flush();

    testConnection();

    flush();
    delay(200);
}

void loop() // run over and over
{
    readData();
    updateData();
    moveCamera();

    /*
    Serial.println("Recieved 1");
    receive_pressure = readFloat();
    Serial.print("recieve_pressure : ");
    Serial.println(receive_pressure);
    mesure_pressure = myPressure.readAltitude(0x60);
    Serial.print("mesure_pressure : ");
    Serial.println(mesure_pressure);
    diff_pressure = receive_pressure - mesure_pressure;
    Serial.print("diff_pressure : ");
    Serial.println(diff_pressure);

    myservoVertical.write(parse_MinMax(57.32*(1.57 - atan(diff_pressure/DISTANCE)), 10, 170));
    */

    delay(50);
}

int parse_MinMax(int val, int mini, int maxi)
{
    return (val > maxi) ? maxi : (val < mini) ? mini : val;
}

//**********************************
//Données requise:
//
//longA, latiA, altiA
//longB, latiB, altiA
//
//A est le recepteur, B le sportif
//**********************************

#define R 6371000

//Everything in rad
//Everything in meters

/* A is the receiver */

int distance;
float angleVertical;
float bearing;

float computeBearing() {
    float longA, latiA, altiA;
    float longB, latiB, altiB;

    longA = dataReceived[1];
    latiA = dataReceived[2];
    altiA = dataReceived[0];

    longB = 0;
    latiB = 0;
    altiB = 0;

    //*** using haversine formula to solve for distance ****
    float RHS = 1 - cos(latiB - latiA) + cos(latiB)*cos(latiA)*(1-cos(longB-longA));

    distance = (int)(R*acos(1 - RHS));
    angleVertical = atan((altiB-altiA)/distance);
    bearing = atan2(sin(longB-longA)*cos(latiB), cos(latiA)*sin(latiB) - cos(longB-longA)*sin(latiA)*cos(latiB));
}
