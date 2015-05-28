/*
La carte cible attend un ordre d'émission infrarouge
par la carte Caméra (lettre 'I').
A la réception de cet ordre, la carte Cible fournit une salve
d'infrarouge pendant une durée imposée par la carte Caméra.
En effet, la carte Caméra ordonne la  fin d'émission infrarouge
par la transmission d'un ordre (lettre 'E').

Cet exemple illustre un scénario complet de commande de l'émetteur
infrarouge par la carte Caméra et met en évidence la synchronisation
nécessaire entre les cartes Cible et Caméra.
*/

#include <arduino.h>

#define LED3  6


void setup()
{
  // XBee module serial configuration
  Serial.begin(9600);
  // LED utilisé pour dire que le signal PWM a été généré
  pinMode(LED3, OUTPUT);
  digitalWrite(LED3, LOW);
}

void loop()
{
  unsigned long time;
  unsigned int count;

  while ((Serial.available() == 0) & (count++ < 10000));
  if (Serial.available() != 0)
  {
    char code = Serial.read();
    // Code = 'I' :  start infrared emission
    if (code == 'I')
    {
      // Generate PWM signal (50 kHz - Duty cycle 30%)
      generatePWM(50000, 30);
      digitalWrite(LED3, HIGH);
      time = millis();
    }
    // Code = 'E' :  stop infrared emission
    if (code == 'E')
    {
      // Stop PWM signal
      generatePWM(50000, 0);
    }
  }
  
  // If code 'E' is not received, stop infrared emssion after 100 ms
  time = millis() - time;
  if (time > 100)
  {
    // Stop PWM signal
    generatePWM(50000, 0);
    digitalWrite(LED3, LOW);
  }
}

// ************************************************************
// Function generatePWM()
// Action : Generate a PWM signal
// Arguments :
//    frequency : PWM signal frequency
//    dutyCycle : Duty cycle of the PWM signal, in % (from 0 to 100)
//                dutyCycle = 0 stops the PWM generation
void generatePWM(long frequency, int dutyCycle)
{
  float temp;
  pinMode(3, OUTPUT);
  if (dutyCycle == 0)
  {
    digitalWrite(3, LOW);
  }
  else
  {
    //Set Timer2 Phase Correct PWM Mode
    // Counter resets when reaching OCRA
    // Output sets while counter < OCRB
    TCCR2A = 0x21;
    TCCR2B = 0x09;
    temp = 16e6 / 2 / frequency;
    OCR2A = int(temp);
    temp = temp * dutyCycle / 100 + 0.5;
    OCR2B = int(temp);
  }
}

