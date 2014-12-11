/* Very first setup sample for GPS */
/* Using GPS module: Ultimate GPS Breakout v3 */

/* Please find original source of library "TinyGPS++" here
   https://github.com/mikalhart/TinyGPSPlus */
/* FYI: The module provider's supporting library could be found
   here. https://github.com/adafruit/Adafruit-GPS-Library */

#include "SoftwareSerial.h"
#include "TinyGPS++.h"

// #include "Adafruit_GPS.h"

#define RXPin 11
#define TXPin 10
#define GPSBaud 9600
#define ConsoleBaud 115200

// #define DEBUG

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
TinyGPSPlus gps;

void setup()
{
  Serial.begin(ConsoleBaud);
  ss.begin(GPSBaud);
  Serial.println("GPS Start");
  Serial.println();
}

void loop()
{
#ifdef DEBUG
  if (ss.available() <= 0) {
    Serial.println("not available...");
  }
#endif

  while (ss.available() > 0) { // As each character arrives...
    char t = ss.read();
    Serial.print(t);
    gps.encode(t);
  }
  Serial.println();

  // if (gps.location.isUpdated() || gps.altitude.isUpdated()) {
  if (gps.location.isValid()) {
    Serial.print("Location: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.print(gps.location.lng(), 6);
    Serial.print("  Altitude: ");
    Serial.println(gps.altitude.meters());
  }

  if (gps.time.isUpdated()) {
    char buf[80];
    sprintf(buf, "The time is %02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.println(buf);
  }

  delay(500);
}

