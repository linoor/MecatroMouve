#include <SoftwareSerial.h>
#include "TinyGPS++.h"
#include <EEPROM.h>

#define RXPin 10
#define TXPin 11
#define GPSBaud 9600
#define ConsoleBaud 115200

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
TinyGPSPlus gps;

bool isConnected = false;
double prevLat = 0, prevLng = 0, prevAlt = 0;
double curLat = 0, curLng = 0, curAlt = 0;
double movingThres = 100;
double prevTime = 0;

int address = 0;

double targetLat = 48.818183, targetLng = 2.338222;
double origLat = 48.817947, origLng = 2.337930;
bool leftOrig = false;

void setup()
{
  Serial.begin(ConsoleBaud);
  ss.begin(GPSBaud);
  Serial.println();
  Serial.println("GPS Start");
  Serial.println(millis());
  Serial.println();

  // EEPROM.write(address, 123);
}

void loop()
{
  while (ss.available() > 0) { // As each character arrives...
    char t = ss.read();
    gps.encode(t);
  }

  if (gps.location.isValid()) {
  // if (gps.location.isUpdated() || gps.altitude.isUpdated()) {
    if (!isConnected) {
        // Serial.print("Cold Start time: ");
        // Serial.println(millis());
        Serial.println("GPS connected");
        isConnected = true;
    }
    curLat = gps.location.lat();
    curLng = gps.location.lng();
    curAlt = gps.altitude.value(); // will return in cm
    // curAlt = gps.altitude.meters();

    double dist = TinyGPSPlus::distanceBetween(curLat, curLng, origLat, origLng);
    Serial.println(dist);

    // if (!leftOrig) {
    //   double dist = TinyGPSPlus::distanceBetween(curLat, curLng, origLat, origLng);
    //   if (dist > 3) {
    //     Serial.println("-------- LEAVE ORIGIN--------");
    //     leftOrig = true;
    //   }
    //   Serial.println(dist);
    // }
    // else {
    //   double dist = TinyGPSPlus::distanceBetween(curLat, curLng, targetLat, targetLng);
    //   if (dist < 3) {
    //     Serial.println("-------- ENTER TARGET--------");
    //   }
    //   Serial.println(dist);
    // }

    // double dist = calDistance(curLat, prevLat, curLng, prevLng);
    prevLat = curLat;
    prevLng = curLng;
    prevAlt = curAlt;
    // Serial.print("Moving distance: ");
    // Serial.println(dist, 6);


    // Serial.print("Location: ");
    // Serial.print(curLat, 6);
    // Serial.print(",");
    // Serial.print(curLng, 6);
    // Serial.print("  Altitude: ");
    // Serial.println(curAlt);
  }
  delay(500);
}
  // if (gps.time.isUpdated()) {
  //   char buf[80];
  //   sprintf(buf, "The time is %02d:%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
  //   Serial.println(buf);
  // }

double calDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x1-x2, 2)+pow(y1-y2, 2));
}