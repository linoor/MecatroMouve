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
2. Download and import required libraries (see below)
3. Set `GPSRXPin` `GPSTXPin` values according to circuit (`GPSRXPin` connects the ***TX*** pin of GPS, and vice versa)
4. Set ports for Servos as first parameters in
  `myservoVertical.attach(PORT_V, 500, 2400)` `myservoHorizontal.attach(PORT_H, 500, 2400)`
5. Upload script to Arduino board
  ***Remember to turn jumpers of XBee shields into USB mode*** *[ref](http://electronics.stackexchange.com/questions/25574/xbee-shield-turning-jumper-settings-into-on-off-xbee-usb-manual-switch)*
6. Configure baudrate to `57600` for Serial Monitor

#### Libraries
GPS
* [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus/releases)
* [Adafruit_GPS]https://github.com/adafruit/Adafruit-GPS-Library

Barometer
* [MPL3115A2](https://github.com/adafruit/Adafruit_MPL3115A2_Library)

9DoF
* [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)
* [Adafruit_LSM303_U](https://github.com/adafruit/Adafruit_LSM303DLHC)
* [Adafruit_L3GD20_U](https://github.com/adafruit/Adafruit_L3GD20_U)

## Explanation
### Communication Process
#### XBee Handshaking
* In `sender.ino`:
    ``` cpp
    while (true)
    {
        Serial.print("A");
        delay(50);
        while (Serial.available())
        {
            // Serial.println("Available");
            if (Serial.read() == 'B')
            {
                // Serial.println("Read B");
                Serial.println("Setup finished!");
                return;
            }
        }
    }
    ```

* In `receiver.ino`:
    ``` cpp
    while (true)
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
    }
    ```

1. The *sender* sends out `A` continuously.
2. Once the *receiver* receives `A`, it returns `B` for 10 times and leaves the session.
3. Once the *sender* receives `B`, handshaking ends.

Note:
* Pass the parameter to `Serial.print`(or `write`) as `string`, while

#### Sending Data

The transmission of data between XBees is facilited with C++ data type `union`.

For example:

```cpp
union bytes
{
    float f;
    byte b[sizeof(float)];
}
```

Variables `f` and `b` actually point to the same memory block (4 bytes in this case), which allows us to manipulate the data as `float` number, and to send data as byte over XBee.

```cpp
bytes data;
// manipulation of data
// e.g. data.f = myPressure.readAltitude();

Serial.write(data.b, sizeof(float));
```

To handle each ideal data type of the sensor, we thereby implement this part with C++ template.

Each data set should be sent at each refresh with a starting signal, a data type signal, and an ending signal to prevent `receiver` from parsing incorrectly due to data loss.

As in `sendData` function:
``` cpp
template <typename T>
void sendData(bytes<T> dataToSend[], int dataSize, String typeSignal)
{
    Serial.print(START_SIGNAL);
    Serial.print(typeSignal);
    for (int i = 0; i < dataSize; i++)
    {
        Serial.write(dataToSend[i].b, sizeof(T));
    }
    Serial.print(END_SIGNAL);
}
```
The current setting of protocal follows
* `START_SIGNAL`: s
* `END_SIGNAL`: e
* `typeSignal`: i, l, f, a, d (as `int`, `long`, `float`, altitude, debug)


### Moving Camera
#### Sender
#### Receiver
#### Formula

## TODO
* moving camera part (barometer first, then self-calibration of receiver, and finally with GPS location)
* optimize handshaking
* redesign start/end protocols for each data transmission

*updated by Eric Chiu 24.02.15*
