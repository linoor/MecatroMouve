// #define PI 3.141592654

double bearingOrig;

double computeBearing(int32_t loc1[], int32_t loc2[])
{
    double lat1 = (double)loc1[0] / 10000000.0;
    double lng1 = (double)loc1[1] / 10000000.0;
    double lat2 = (double)loc2[0] / 10000000.0;
    double lng2 = (double)loc2[1] / 10000000.0;
    // return atan((lat2-lat1)/cos(lat1*PI/180)/(lng2-lng1))*180/PI;
    /*Serial.print("Loc1: ");Serial.print(lat1 * 10000000);Serial.print(" ");Serial.println(lng1 * 10000000);
    Serial.print("Loc2: ");Serial.print(lat2 * 10000000);Serial.print(" ");Serial.println(lng2 * 10000000);*/
    return atan((lng2-lng1)/(lat2-lat1)) * 180 / PI;
}

double computeVertical(int32_t loc1[], int32_t loc2[], float alti1, float alti2)
{
    double lat1 = (double)loc1[0] / 10000000.0;
    double lng1 = (double)loc1[1] / 10000000.0;
    double lat2 = (double)loc2[0] / 10000000.0;
    double lng2 = (double)loc2[1] / 10000000.0;
    double dist = sqrt(pow(lat2-lat1, 2) + pow(lng2-lng1, 2));
    return atan((alti1-alti2)/dist)*180/PI;
}

double realAngle(double bearingCur)
{
    switch(bearingState)
    {
    case 1:
        if (bearingCur < 0)
        {
            bearingState = 2;
            return 180 + bearingCur;
        }
        return bearingCur;
        break;
    case 2:
        if (bearingCur > 0)
        {
            bearingState = 3;
            return 180 + bearingCur;
        }
        return 180 + bearingCur;
        break;
    case 3:
        if (bearingCur < 0)
        {
            bearingState = 4;
            return 360 + bearingCur;
        }
        return 180 + bearingCur;
        break;
    case 4:
        if (bearingCur > 0)
        {
            if (bearingOrig)
            bearingState = 1;
            return bearingCur;
        }
        return 360 + bearingCur;
        break;
    }
}

double updateBearing(int32_t loc1[], int32_t loc2[])
{
    double bearing = computeBearing(loc1, loc2);
    // Serial.println(bearing);
    if (!isBearingInit)
    {
        bearingOrig = bearing;
        isBearingInit = true;
        Serial.print("Bearing initialized");
    }
    else
    {
        bearing = realAngle(bearing);
        bearingOrig = bearing;
    }
    Serial.println(bearing);
    return bearing;
}

