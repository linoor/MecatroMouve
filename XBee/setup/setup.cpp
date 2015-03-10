void setupBaro(MPL3115A2 myPressure)
{
    myPressure.begin();

    myPressure.setModeAltimeter();
    myPressure.setOversampleRate(6); // Pour lire 1 seule valeur, il lui faut 512ms
    // Du coup pas besoin de moyenner quoique ce soit!
    myPressure.enableEventFlags();
}

void flush()
{
    while (Serial.available())
    {
        Serial.read();
    }
}
