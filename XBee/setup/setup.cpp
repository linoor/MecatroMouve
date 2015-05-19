/* Functions in this file are called by both receiver and sender */

#ifndef DEF_H
#include "def.h"
#endif
#include "I2C.h"
#include "BMP180.h"
#ifndef KALMAN_H
#define KALMAN_H
    kalman_state altitude_state;
#endif
    
#define address 0x1E //0011110b, I2C 7bit address of HMC5883

byte OverSample = 0;

// flush current buffer
void flush()
{
    while (Serial.available())
    {
        Serial.read();
    }
}

// setup "myPressure" variable for barometer
void setupBaro()
{
    Serial.println("Setting up barometer");
    BMP180_Init(OverSample);
    if (I2C_Read(BMP180_ADDRESS, BMP180_ID) == 0x55)
    {
        Serial.println("BMP180 ready ...");
    }
    else
    {
        Serial.println("BMP180 not connected ...");
    }
    float temperature, pressure, altitude;
    BMP180_getMeasurements(temperature, pressure, altitude);
    altitude_state = kalman_init(0.0001, 10, 10, altitude);
    kalman_state altitude_state;
}

//////////////// GPS setting ////////////////
// **copied from Adafruit_GPS parse example**

void setupGPS()
{
    Serial.println("Setting up GPS");
    GPS.begin(GPSBaud);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);
    useInterrupt(true);
    Serial.println("GPS ready");
}

/////////////// Magnetometre ////////////////

//Put the HMC5883 IC into the correct operating mode
void setupMagnetometre() {
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission(); 
}

struct MagnetometreData {
    int x;
    int y;
    int z;
};

MagnetometreData readMagnetometre() {
  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();

  Wire.requestFrom(address, 6);

  MagnetometreData result;

  if(6<=Wire.available()){
    result.x = Wire.read() << 8; //X msb
    result.x |= Wire.read(); //X lsb
    result.z = Wire.read() << 8; //Z msb
    result.z |= Wire.read(); //Z lsb
    result.y = Wire.read() << 8; //Y msb
    result.y |= Wire.read(); //Y lsb
  }

  return result;
}

////////////// End Magnetometre ////////////////

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

void useInterrupt(bool v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void getGPSLocation(int32_t* location)
{
    if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trying to print out data
        //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
            return;  // we can fail to parse a sentence in which case we should just wait for another
    }

    // Serial.print("Fix: "); Serial.print((int)GPS.fix);
    //Serial.print("Number of satellites: ");
    //Serial.println(GPS.satellites);
    if (GPS.fix)
    {
        location[0] = GPS.latitude_fixed;
        location[1] = GPS.longitude_fixed;
    }
    else
    {
        //Serial.println("skip GPS update...");
    }

    // Serial.print("Location: ");
    // Serial.print(GPS.latitude_fixed); Serial.print(GPS.lat);
    // Serial.print(", ");
    // Serial.print(GPS.longitude_fixed); Serial.println(GPS.lon);
}

void getGPSLocation(double* location) {}
