// Routine transmission Alexmos RS232 via Arduino

#define SBGC_CMD_CONTROL 67
/* ***to be included in the cpp file:
#include <inttypes.h>
#include <SoftwareSerial.h>

SoftwareSerial portAlex(6,7);// RX, TX
#include "Alexmos.h"
*/

//***in cpp file, use Alex_updateBearing followed by
//***Alex_createPackage, with current and desired cap as input, respectively

//*****must be initialised and updated constantly*****
int camera_Roll = 0;
int camera_Pitch = 0;
int camera_Yaw = 0;

//**** RateOfUpdate
int updateTime = 200;  //in milliseconds

// CMD_CONTROL structure
typedef struct {
    uint8_t mode;
    int16_t speedROLL;
    int16_t angleROLL;
    int16_t speedPITCH;
    int16_t anglePITCH;
    int16_t speedYAW;
    int16_t angleYAW;
} SBGC_Data_Packet;

void Alex_updateBearing(int roll, int pitch, int yaw) {
    //update the direction of the camera AFTER movement
    camera_Pitch = pitch;
    camera_Yaw = yaw;
    camera_Roll = roll;
}


//SendCommand Enoi de la trame vers alexmos
void Alex_sendCommand(uint8_t cmd, void *data, uint16_t size)
{   uint8_t i, checksum = 0;
    // Header
    portAlex.write('>');
    portAlex.write(cmd);
    portAlex.write(size);
    portAlex.write(cmd + size);
    // Body
    for (i = 0; i < size; i++) {
        checksum += ((uint8_t*)data)[i];
        portAlex.write(((uint8_t*)data)[i]);
    }
    portAlex.write(checksum);
}

// function de conversion
int16_t DEGREE_TO_ANGLE(int d) {return (int16_t)(d / 0.02197265625);}
int ANGLE_TO_DEGREE(int16_t d) {return (int)(d * 0.02197265625);}

//input: the bearing of the target with respect to the NORTH   //****All inputs in integer degree****
//assumed: knowing the current direction of the camera w.r.t the NORTH
void Alex_createPackage(int roll, int pitch, int yaw) {
    SBGC_Data_Packet param = { 0, 0, 0, 0, 0, 0, 0 };
    param.mode = 2;
    //param.anglePITCH = DEGREE_TO_ANGLE(pitch);
    param.angleYAW = DEGREE_TO_ANGLE(yaw);
    //param.angleROLL = DEGREE_TO_ANGLE(roll);
    // unit of speed used is 0.1220740379 degree/sec
    //param.speedPITCH = int16_t(abs(pitch - camera_Pitch) / 0.1220740379 / (updateTime / 1000.0));
    //param.speedROLL = int16_t(abs(roll - camera_Roll) / 0.1220740379 / (updateTime / 1000.0));
    // param.speedYAW = int16_t(abs(yaw - camera_Yaw) / 0.1220740379 / (updateTime / 1000.0));
    param.speedYAW = 500;

    Serial.print("Yaw: ");
    Serial.print(yaw);
    // Serial.print(" ");
    // Serial.println(param.angleYAW);
    // Serial.println(param.speedYAW);

    Alex_sendCommand(SBGC_CMD_CONTROL, &param, sizeof(param));
    Alex_updateBearing(roll, pitch, yaw);
}
