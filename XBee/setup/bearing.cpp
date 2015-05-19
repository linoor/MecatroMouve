#define PI 3.141592654

double computeBearing(int32_t loc1[], int32_t loc2[])
{
    double lat1 = (double)loc1[0] / 10000000;
    double lng1 = (double)loc1[1] / 10000000;
    double lat2 = (double)loc2[0] / 10000000;
    double lng2 = (double)loc2[1] / 10000000;
    return atan((lat2-lat1)/cos(lat1*PI/180)/(lng2-lng1))*180/PI;
}

double computeVertical(int32_t loc1[], int32_t loc2[], float alti1, float alti2)
{
    double lat1 = (double)loc1[0] / 10000000;
    double lng1 = (double)loc1[1] / 10000000;
    double lat2 = (double)loc2[0] / 10000000;
    double lng2 = (double)loc2[1] / 10000000;
    double dist = sqrt(pow(lat2-lat1, 2) + pow(lng2-lng1, 2));
    return atan((alti1-alti2)/dist)*180/PI;
}