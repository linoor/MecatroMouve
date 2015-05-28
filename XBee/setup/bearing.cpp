// #define PI 3.141592654

// this is not the real angle but a cache of the last result from computeBearing
double bearingOrig;

// each turn should be multiply by 360
int cameraTurns = 0;

// compute the bearing angle of loc1 and loc2
// loc1: received gps = target
// loc2: my gps = camera
double computeBearing(int32_t loc1[], int32_t loc2[])
{
    double lat1 = (double)loc1[0] / 10000000.0;
    double lng1 = (double)loc1[1] / 10000000.0;
    double lat2 = (double)loc2[0] / 10000000.0;
    double lng2 = (double)loc2[1] / 10000000.0;
    // Serial.print("--");Serial.println(atan2((lng1-lng2), (lat1-lat2)) * 180 / PI);
    return atan2((lng1-lng2), (lat1-lat2)) * 180 / PI;
    // return atan((lng2-lng1)/cos(lng1*PI/180)/(lat2-lat1)) * 180 / PI;
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

double realAngle(double bearingDelta)
{
    if (bearingDelta < -180)
    {
        cameraTurns++;
    }
    else if (bearingDelta >= -180 && bearingDelta  < 0)
    {

    }
    else if (bearingDelta >= 0 && bearingDelta  < 180)
    {
    }
    else if (bearingDelta >= 180)
    {
        cameraTurns--;
    }
    else {}
    double bearingDeltaReal = bearingDelta + cameraTurns * 360;

    return bearingOrig + bearingDeltaReal;

    /*switch(bearingState)
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
    }*/
}

double updateBearing(int32_t loc1[], int32_t loc2[])
{
    double bearing = computeBearing(loc1, loc2);
    // real bearing angle to move the motor
    double bearingReal;

    if (!isBearingInit)
    {
        bearingReal = bearingOrig = bearing;
        isBearingInit = true;
        // Serial.print("Bearing initialized");
    }
    else
    {
        // Serial.print(bearing); Serial.print(" "); Serial.println(bearingOrig);
        double bearingDelta = bearing - bearingOrig;
        bearingReal = realAngle(bearingDelta);
        bearingOrig = bearing;
        Serial.println(bearingReal);
    }
    return bearingReal;
}

