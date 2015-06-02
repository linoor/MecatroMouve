// Routine transmission Alexmos RS232 via Arduino

#define SBGC_CMD_CONTROL 67


// CMD_CONTROL structure
typedef struct {
  uint8_t mode;
  int16_t speedROLL;
  int16_t angleROLL;
  int16_t speedPITCH;
  int16_t anglePITCH;
  int16_t speedYAW;
  int16_t angleYAW;
} SBGC_cmd_control_faul;

//SendCommand Enoi de la trame vers alexmos
void FAUL_sendCommand(uint8_t cmd, void *data, uint16_t size) 
{  uint8_t i, checksum=0;
	  // Header
	  portAlex.write('>');
	  portAlex.write(cmd);
	  portAlex.write(size);
	  portAlex.write(cmd+size);
	  // Body
	  for(i=0;i<size;i++) {
		checksum+= ((uint8_t*)data)[i];
		portAlex.write(((uint8_t*)data)[i]);
	  }
	  portAlex.write(checksum);
 }
 
 // function de conversion
float FAUL_DEGREE_TO_ANGLE(int d) {return d/0.02197265625;}
int FAUL_ANGLE_TO_DEGREE(float d) {return d*0.02197265625;}
