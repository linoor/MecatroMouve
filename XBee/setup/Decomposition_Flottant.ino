float fSent, fReceived;
byte tByte[4];
byte temp;

void setup()
{
  Serial.begin(9600);
  // Define float to be transmitted
  fSent = 30.854;

  // Break float into bytes
  long temp = long(fSent * 1000000);
  for (int i = 0; i < 4; i++)
  {
    tByte[i] = byte((temp >> (8 * i)) & 0xFF);
  }


  Serial.print("Flottant envoye : ");
  Serial.println(fSent);

  // Compute the received float
  fReceived = 0;
  temp = 0;
  for (int i = 0; i < 4; i++)
  {
       temp |= long(tByte[i]) << (8 * i);
  }
  fReceived = float(temp)/1000000;
  Serial.print("Flottant recu : ");
  Serial.println(fReceived);


}

void loop() {
  // put your main code here, to run repeatedly:

}
