/*
La carte Caméra fournit à la carte Cible un ordre d'émission
infrarouge (lettre 'I').
A la réception de cet ordre, la carte Cible émet une salve d'infrarouge.
Durant cette salve, la carte Caméra analyse si l'infrarouge a été reçu.
A la fin de cette analyse, la carte Caméra fournir à la carte Cible
un ordre d'arrêt d'émission infrarouge (lettre 'E').

Cet exemple illustre un scénario complet de commande de l'émetteur
infrarouge par la carte Caméra et met en évidence la synchronisation
nécessaire entre les cartes Cible et Caméra.
*/

#include <SPI.h>
#include "infrared.h"
#include "myADC.h"

#define LED3  6
// SCOPE used to timing measurements with a scope
#define SCOPE  3

void setup()
{
  // XBee module serial configuration
  // Must be changed according to XBee module baudrate configuration
  Serial.begin(57600);

  // LED3 used to visualize the infrared reception
  pinMode(LED3, OUTPUT);
  digitalWrite(LED3, LOW);
  
  pinMode(SCOPE, OUTPUT);
  digitalWrite(SCOPE, LOW);

  // Infrared interface initialization
  Init_Infrared();

  // Analog/Digital Converter initialization (A6)
  ADC_init(6);
}

void loop()
{
  char infraOK;
  unsigned char gain1, gain2;
  long maxResult = 0;
  long maxMesure = 0;
  long mesure = 0;
  unsigned char numPin = 0;
  unsigned char i = 0;
  unsigned char vgain1 = 0;
  unsigned char vgain2 = 0;
  // Select one of the infrared cells input (cell numbers : 1 to 10)
  infraOK = 0;

  // Transmit letter 'I' to start infrared emission
  Serial.print('I');
  delay(10);
  
  digitalWrite(SCOPE, HIGH);
  for( i = 1; i<=10;i++)
  {
    infraOK = 0;
    Select_InfraInput(i);
    for (gain2 = 0; gain2 <= 2; gain2++)
    {
      // Configure Amplifier1 gain
      vgain2 = PGA113_GainWrite(2, gain2);
      for (gain1 = 0; gain1 <= 7; gain1++)
      {
        // Configure Amplifier2 gain
        vgain1 = PGA113_GainWrite(1, gain1);
        delayMicroseconds(100);
        // Fast ADC conversion method. ADC input is A6
        ADC_init(6);
        unsigned int result = ADC_get(6);
        Serial.print(" Num - ");
        Serial.print(i);
        Serial.print(" : ");
        Serial.print(result);
        Serial.print(" gain: ")
        Serial.println(vagin1*vgain2);
        // If 0.35V < voltage < 3.5V (no saturation) (Thresholds 300 - 600)
        if ((result > 300) && (result < 600))
        {
          mesure = vgain1*vgain2*result;
          if(mesure > maxMesure){
            maxMesure = mesure;
            maxResult = result;
            numPin = i;
          }
          infraOK = 1;
          break;
        }
      }
      if (infraOK)  break;
    }
  }
  digitalWrite(SCOPE, LOW);

  // Transmit letter 'E' to stop infrared emission
  Serial.print('E');
  // Use LED3 (green led) to visualize the reception
  if (numPin > 0)
  {
    //Serial.println(numPin);
    //Serial.println(maxResult);
    digitalWrite(LED3, HIGH);
  }
  else
  {
    digitalWrite(LED3, LOW);
  }

  // Wait 1 second
  delay(1000);
  digitalWrite(LED3, LOW);
}

