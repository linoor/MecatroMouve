/******************************************************************************
FAUL philippe
alexmos arduino QX10
*******************************************************************************/
#include <inttypes.h>
#include <SoftwareSerial.h>

SoftwareSerial portAlex(6,7);// RX, TX
#include "Alexmos.h"

#include <SPI.h>
#include <boards.h>
#include <RBL_nRF8001.h>

SBGC_cmd_control_faul param = { 0, 0, 0, 0, 0, 0, 0 };

/*****************************************************************************/

void setup() {
 // Serial.begin(115200); // texte ecran
    ble_begin();
    delay(1000);
    portAlex.begin(9600);//Alexmos
  delay(1000);

}

void loop() {
  // Attente des données BLE de l'iphone
   while(ble_available())
  {
    // read out command and data
    byte data0 = ble_read();
    byte data1 = ble_read();
    byte data2 = ble_read();

    // on recoit de l'iphone une valeur entre 0-255 qu'on va recentrer

    if (data0 == 0x02)  // Command is to control digital out pin
    { param.anglePITCH = FAUL_DEGREE_TO_ANGLE((128-data1)/4); }//-32 à 32 degree
    else if (data0 == 0x03)  // Command is to control Servo pin
    {    param.angleYAW = FAUL_DEGREE_TO_ANGLE((data1-128)/2); }//-64 à 64 degree
   }
   ble_do_events();

   // On Balance vers alexmos

  	param.mode = 2; //SBGC_CONTROL_MODE_ANGLE
	param.speedROLL = param.speedPITCH = param.speedYAW = 500;
        //Units: 0,1220740379 degree/sec
	FAUL_sendCommand(SBGC_CMD_CONTROL, &param, sizeof(param));
	delay(100);

}
// This helper function formats and sends a command according to SimpleBGC Serial API specification




