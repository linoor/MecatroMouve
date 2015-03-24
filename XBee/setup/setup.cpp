#define GPSBaud 9600

void setupBaro(MPL3115A2 myPressure)
{
    myPressure.begin();

    myPressure.setModeAltimeter();
    myPressure.setOversampleRate(6); // Pour lire 1 seule valeur, il lui faut 512ms
    // Du coup pas besoin de moyenner quoique ce soit!
    myPressure.enableEventFlags();
}

void setupGPS(SoftwareSerial ss)
{
    ss.begin(GPSBaud);
    Serial.println("GPS started...");
}

void getGPSPosition(SoftwareSerial ss, TinyGPSPlus gps, float *pos)
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

void flush()
{
    while (Serial.available())
    {
        Serial.read();
    }
}
