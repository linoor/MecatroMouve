#include <StandardCplusplus.h>
#include <string>
#include <map>

std::map<char, std::string> m;

const int ledPin = 13; // pin the LED is connected to
int blinkRate = 0;     // blink rate stored in this variable

void setup()
{
  Serial.begin(9600); // Initialize serial port to send and receive at 9600 baud
  pinMode(ledPin, OUTPUT); // set this pin as output
}

void loop()
{
  if (Serial.available()) // Check to see if at least one character is available
  {
    char ch = Serial.read();
    if( isDigit(ch) ) // is this an ascii digit between 0 and 9?
    {
    	int max = charToInt(ch);
      Serial.println(ch);
    	for(int i = 0; i < max; i++) {
    		blink(100);
    	}
    }
  }
}

// blink the LED with the on and off times determined by blinkRate
void blink(int timeDelay) {
  digitalWrite(ledPin,HIGH);
  delay(timeDelay);
  digitalWrite(ledPin, LOW);
  delay(timeDelay);
}

int charToInt(char c) {
	return (int)(c - '0');
}

