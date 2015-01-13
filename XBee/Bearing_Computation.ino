//**********************************
//Données requise:
//
//LongA, LatiA, AltiA
//LongB, LatiB, AltiA
//
//A est le recepteur, B le sportif
//**********************************

#include <math.h>
#define R 6371000

//Everything in rad
//Everything in meters

/* A is the receiver */
float LongA, LatiA, AltiA;
float LongB, LatiB, AltiB;
int distance;
float angleVertical;
float bearing;

float computeBearing() {
//*** using haversine formula to solve for distance ****
float RHS = 1 - cos(LatiB - LatiA) + cos(LatiB)*cos(LatiA)*(1-cos(LongB-LongA));

distance = (int)(R*acos(1 - RHS));
angleVertical = atan((AltiB-AltiA)/distance);
bearing = atan2(sin(LongB-LongA)*cos(LatiB), cos(LatiA)*sin(LatiB) - cos(LongB-LongA)*sin(LatiA)*cos(LatiB));
}

/********
A faire: 
1* initialiser et convertir les données sous les bonnes unités
2* calculer l'angle qu'il faut tourner la caméra basé sur les données gyro-boursol, etc.
*/


