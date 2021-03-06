#ifndef DEF_H
#define DEF_H
#define GPSRXPin 4
#define GPSTXPin 5
#define GPSBaud 9600 // cf documentation
#define AlexRX 6
#define AlexTX 7
#define START_SIGNAL 's'
#define END_SIGNAL 'e'
#define FLOAT_SIZE sizeof(float)

#define PI 3.141592654

SoftwareSerial ss(GPSTXPin, GPSRXPin);
SoftwareSerial portAlex(AlexRX, AlexTX);
// TinyGPSPlus gps;
Adafruit_GPS GPS(&ss);

bool isBearingInit = false;
bool isVertiInit = false;

bool usingInterrupt = false;

int bearingState = 1;

typedef byte uint8_t;

template <typename T>
union bytes
{
    T f;
    byte b[sizeof(T)];
};

void useInterrupt(bool);

#endif
