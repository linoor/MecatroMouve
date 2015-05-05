#ifndef DEF_H
#define DEF_H
#define GPSRXPin 4
#define GPSTXPin 5
#define GPSBaud 9600
#define START_SIGNAL 's'
#define END_SIGNAL 'e'
#define FLOAT_SIZE sizeof(float)

SoftwareSerial ss(GPSTXPin, GPSRXPin);
// TinyGPSPlus gps;
Adafruit_GPS GPS(&ss);

bool isInit = false;

bool usingInterrupt = false;

typedef byte uint8_t;

template <typename T>
union bytes
{
    T f;
    byte b[sizeof(T)];
};

void useInterrupt(bool);

#endif
