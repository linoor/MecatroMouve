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

#include "../setup/def.h"
#include "../setup/setup.cpp"

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

#define IDLE_STEPS 80
#define CALIBRATION_STEPS 20

// #define DEBUG


///////////////////////////////////////
//Les données reçus
float dataReceived[RECEIVE_SIZE];

////////////////////////////////////////
//les données de CET Arduino
float dataCurrent[LOCAL_SIZE];

MPL3115A2 myPressure;

SoftwareSerial ss(GPSRXPin, GPSTXPin);
TinyGPSPlus gps;

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

Servo myservoVertical, myservoHorizontal;
float receive_pressure, mesure_pressure, diff_pressure;

//Everything in rad
//Everything in meters

/* A is the receiver */
int angleTest = 0;
int distance;
float angleVertical;
float bearing;
const float Pi = 3.14159;
float myAlti;
float correction = 0; //Correction factor to apply to compute altitude differential.

////////// setting things up //////////

void setupGPS()
{
    ss.begin(GPSBaud);
    Serial.println("GPS started...");
}

void setupAccMagGyro()
{
    /* Initialise the sensor */
    if (!accel.begin() || !mag.begin())
    {
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        // while (1);
    }
    Serial.println("Acc + Mag started...");

    if (!gyro.begin())
    {
        Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
        // while (1);
    }
    Serial.println("Gyro started...");
}

void setupServo()
{
    myservoVertical.attach(10, 500, 2400); // Attach servos
    myservoHorizontal.attach(11, 500, 2400);
    myservoVertical.write(0);
    myservoHorizontal.write(0);
}

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

////////// handling data //////////

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
    else {
        pos[0] = -1;
        pos[1] = -1;
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


////////////////////////////////////////
//mettre à jour les donnée de CET arduino

void printDataCurrent()
{
    Serial.print("Altitude (Baro) : "); Serial.println(dataCurrent[0]);
    Serial.print("Longitude (GPS) : "); Serial.println(dataCurrent[1]);
    Serial.print("Latitude  (GPS) : "); Serial.println(dataCurrent[2]);
    Serial.print("Acc  x   (9DoG) : "); Serial.println(dataCurrent[3]);
    Serial.print("Acc  y   (9DoG) : "); Serial.println(dataCurrent[4]);
    Serial.print("Acc  z   (9DoG) : "); Serial.println(dataCurrent[5]);
    Serial.print("Mag  x   (9DoG) : "); Serial.println(dataCurrent[6]);
    Serial.print("Mag  y   (9DoG) : "); Serial.println(dataCurrent[7]);
    Serial.print("Mag  z   (9DoG) : "); Serial.println(dataCurrent[8]);
    Serial.print("Gyro x   (9DoG) : "); Serial.println(dataCurrent[9]);
    Serial.print("Gyro y   (9DoG) : "); Serial.println(dataCurrent[10]);
    Serial.print("Gyro z   (9DoG) : "); Serial.println(dataCurrent[11]);
}

void updateData()
{
    Serial.print("Updating data... My altitude: ");
    // Serial.print(myPressure.readAltitude());
    float gpsPosition[2];
    float accMagGyro[9];

    for(int i = 0; i < 9; i++){
        accMagGyro[i] = 0;
    }

    //getGPSPosition(gpsPosition);
    // getAccMagGyro(accMagGyro);

    dataCurrent[0] = myPressure.readAltitude();
    myAlti = myPressure.readAltitude();
    Serial.println(myAlti);
    dataCurrent[1] = gpsPosition[0];
    dataCurrent[2] = gpsPosition[1];
    dataCurrent[3] = accMagGyro[0];
    dataCurrent[4] = accMagGyro[1];
    dataCurrent[5] = accMagGyro[2];
    dataCurrent[6] = accMagGyro[3];
    dataCurrent[7] = accMagGyro[4];
    dataCurrent[8] = accMagGyro[5];
    dataCurrent[9] = accMagGyro[6];
    dataCurrent[10] = accMagGyro[7];
    dataCurrent[11] = accMagGyro[8];

    //printDataCurrent();
}

////////////////////////////////////////
//bouge les servos!!!!!
//**********************************
// Données requise:
//
// longA, latiA, altiA
// longB, latiB, altiA
//
// A est le recepteur, B le sportif
//**********************************
// Everything in rad
// Everything in meters

void moveCamera()
{
    float longA, latiA, altiA;
    float longB, latiB, altiB;

    // hard-coded version
    longA = 0;
    latiA = 0;
    altiA = 0;
    // angleTest += 30;
    // longB = cos(angleTest);
    // latiB = sin(angleTest);
    longB = 0;
    latiB = 0.002;
    // distance = 5;

    // longA = dataReceived[1];
    // latiA = dataReceived[2];
    altiA = dataCurrent[0];

    // longB = dataCurrent[1];
    // latiB = dataCurrent[2];
    altiB = dataReceived[0];

    //*** using haversine formula to solve for distance ****
    float RHS = 1 - cos(latiB - latiA) + cos(latiB)*cos(latiA)*(1-cos(longB-longA));

    // distance = (int)(R*acos(1 - RHS));
    angleVertical = atan((altiB-altiA)/distance);
    bearing = atan2(sin(longB-longA)*cos(latiB), cos(latiA)*sin(latiB) - cos(longB-longA)*sin(latiA)*cos(latiB));

    float servoVert = 90 - angleVertical/Pi*180;
    float servoHoriz = bearing/Pi*180;
    myservoVertical.write(servoVert);
    myservoHorizontal.write(servoHoriz);

#ifdef DEBUG
    Serial.print("Alti A: "); Serial.println(altiA);
    Serial.print("Alti B: "); Serial.println(altiB);
    Serial.print("Vertical: "); Serial.println(servoVert);
    Serial.print("Horizontal: "); Serial.println(servoHoriz);
    Serial.println();
#endif
}

////////////////////////////////////////
//recevoir les données

float readFloat()
{
    bytes<float> payload;

    while (!Serial.available());

    for (int i = 0; i < FLOAT_SIZE; i++)
    {
        payload.b[i] = Serial.read();
        // delay(5);
    }

    return payload.f;
}

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
            // printDataReceived();
        }
        Serial.println();
    }
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

template <typename T>
T readSingleData() {
    bytes<T> received;
    for (int i = 0; i < sizeof(T); i++)
    {
        received.b[i] = Serial.read();
    }
    return received.f;
}
int counter = 0;

void readTestData()
{
    if (!Serial.available()) return;

    while (Serial.read() != START_SIGNAL)
    {
    }
    // delay(100);

    float alti;
    int32_t gpsPosition[2];

    long testLong;
    int testInt;

    switch (Serial.read())
    {
        case 'a':
            Serial.println(counter);
            alti = readDataTest<float>();
            if(counter < IDLE_STEPS){
                counter++;
            }
            else{
              if(counter - IDLE_STEPS < CALIBRATION_STEPS){
                correction += (alti - myAlti);
                counter++;
                }
                else if(counter - IDLE_STEPS == CALIBRATION_STEPS) {
                    correction = correction/CALIBRATION_STEPS;
                    counter++;
                    Serial.println(correction);
                }
            }
            break;
        case 'l': // test for sending long
            Serial.println("l");
            testLong = readDataTest<long>();
            break;
        case 'i':
            Serial.println("i");
            testInt = readDataTest<int>();
            break;
        case 'd':
            Serial.println("Start looking for end signal");
            while(Serial.read() != END_SIGNAL){
                //Serial.println("Not received yet... going on");
            }
            Serial.println("Found end signal. Returning");
            return;
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
        Serial.print("Alti received: ");
        Serial.println(alti);
        Serial.print("Differential: ");
        Serial.println(alti - myAlti - correction);
    }
}

////////////////////////////////////////

void setup()
{
    Wire.begin();

    setupBaro(myPressure);
    // setupGPS();
    // setupAccMagGyro();
    // setupServo();

    Serial.begin(57600);
    flush();

    receiverConnect();

    flush();
    delay(500);
  //  calibrateAltitude();

}

int loopCounter = 0;

void loop() // run over and over
{
    if(loopCounter == 0){
        updateData();
    }
    loopCounter = (loopCounter+1)%10;
    // readData();
    // moveCamera();
    /*myservoVertical.write(parse_MinMax(57.32*(1.57 - atan(diff_pressure/DISTANCE)), 10, 170));
    */
    readTestData();
    delay(300);
}

