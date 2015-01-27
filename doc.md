GoMouve
=======
**prototype version 0.1**

This document provides a tutorial on how to set up GoMouve prototype, and explains the mechanism behind it.

## Introduction

## Set Up
#### Components
For *sender* and *receiver*, the following devices are required:
##### Sender
* Arduino Uno * 1
* XBee & shield * 1
* GPS (with antenna) * 1
* Barometer * 1

##### Receiver
* Arduino Uno * 1
* XBee & shield * 1
* GPS (with antenna) * 1
* Barometer * 1
* 9DOF * 1
* Servo * 2

*Current models: [XBee & shield](http://www.cooking-hacks.com/shop/arduino/arduino-xbee-802-15-4), [GPS](http://www.adafruit.com/product/746), [Barometer](http://www.adafruit.com/product/1893) (w/o* ***3Vo*** *pin), [9DOF](http://www.adafruit.com/product/1714), [Servo](http://www.miniplanes.fr/servos/tower-pro/mini-servo-9g-towerpro-sg90-p-2995.html)*

#### Wiring
The sketch of *receiver* gives an idea on how to wire up all components.
![Alt text](https://dl.dropboxusercontent.com/u/17953813/receiver.png "receiver's sketch")
Note:
* Check the ports for ***SCL*** and ***SDA*** of your Arduino board

#### Uploading Scripts
1. Get scripts of *sender* and *receiver* from [here](https://github.com/linoor/MecatroMouve/tree/master) (each in `XBee/sender`,  `XBee/receiver`)
2. Download and import required libraries
3. Set `GPSRXPin` `GPSTXPin` values according to circuit (`GPSRXPin` connects the ***TX*** pin of GPS, and vice versa)
4. Set ports for Servos as first parameters in
  `myservoVertical.attach(PORT_V, 500, 2400)` `myservoHorizontal.attach(PORT_H, 500, 2400)`
5. Upload script to Arduino board
  ***Remember to turn jumpers of XBee shields into USB mode*** *[ref](http://electronics.stackexchange.com/questions/25574/xbee-shield-turning-jumper-settings-into-on-off-xbee-usb-manual-switch)*
6. Configure baudrate to `57600` for Serial Monitor

## Explanation
### Communication Process
#### XBee Handshaking
###### Example
* In `sender.ino`:  
``while (true)
{
    // Start connecting...
    Serial.print("A");
    delay(50);
    while (Serial.available())
    {
        if (Serial.read() == 'B')
        {
            // Setup finished!
            return;
        }
    }
}``

* In `receiver.ino`:  
``while (true)
{
    delay(10);
    if (Serial.available())
    {
        if (Serial.read() == 'A')
        {
            for (int i = 0; i < 10; i++)
            {
                Serial.print("B");
            }
            return;
        }
    }
}``

#### Sending Data

### Moving Camera
#### Sender
#### Receiver
#### Formula

## TODO
* moving camera part (barometer first, then self-calibration of receiver, and finally with GPS location)
* optimize handshaking
* redesign start/end protocols for each data transmission

*updated by Eric Chiu 26.01.15*
