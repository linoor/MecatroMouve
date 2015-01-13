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
float dataCurrent[LOCAL_SIZE];

MPL3115A2 myPressure;

SoftwareSerial ss(GPSRXPin, GPSTXPin);
TinyGPSPlus gps;

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

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

// void testSensors()
// {
//     Serial.println(myPressure.readAltitude());

//     float myGPSPosition[2];
//     getGPSPosition(myGPSPosition);
//     Serial.print(myGPSPosition[0]);
//     Serial.print(",");
//     Serial.print(myGPSPosition[1]);
//     Serial.println();

//     float accMagGyro[9];
//     getAccMagGyro(accMagGyro);
//     for(int i = 0; i < 9; i++) {
//         Serial.print(accMagGyro[i]);
//         Serial.print(",");
//     }
//     Serial.println();
// }

////////////////////////////////////////
//mettre à jour les donnée de CET arduino
void updateData()
{
    Serial.print(myPressure.readAltitude());

    float gpsPosition[2];
    float accMagGyro[9];

    getGPSPosition(gpsPosition);
    getAccMagGyro(accMagGyro);

    dataCurrent[0] = myPressure.readAltitude();
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

    printDataCurrent();
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
            Serial.println("START_SIGNAL not received...");
        }
        Serial.println("START_SIGNAL success!!");
        delay(50);

        // Receiving RECEIVE_SIZE floats
        float temp[RECEIVE_SIZE];
        for (int i = 0 ; i < RECEIVE_SIZE ; i++)
        {
            temp[i] = readFloat();
            // delay(10);
        }
        // delay(50);

        // Wait for END_SIGNAL
        if (Serial.read() != END_SIGNAL)
        {
            Serial.println("END_SIGNAL not received...");
            // Truncate data if END_SIGNAL not received
        }
        else
        {
            Serial.println("END_SIGNAL success!!");
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

////////////////////////////////////////

void setup()
{
    Wire.begin();

    setupBaro();
    setupGPS();
    setupAccMagGyro();
    setupServo();

    Serial.begin(9600);
    flush();

    // testConnection();

    flush();
    delay(200);
}

void loop() // run over and over
{
    updateData();
    // readData();
    moveCamera();

    // testSensors();

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

    delay(100);
}

int parse_MinMax(int val, int mini, int maxi)
{
    return (val > maxi) ? maxi : (val < mini) ? mini : val;
}
