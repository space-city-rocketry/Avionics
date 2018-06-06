#ifndef CDH_h
#define CDH_h
#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <TimeLib.h>

#define XBEE Serial1
#define GPS Serial2

class CDH
{
  public:
    //BMP180 data variables:
    //example: int bmpAltitude;
    //BNO055 data variables:

    //other data variables/objects:
    Adafruit_BNO055 bno = Adafruit_BNO055(55);
    Adafruit_BMP085 bmp;
    TinyGPS gps

    //Detections
    bool launch = false;
    bool apogee = false;
    bool main = false;
    bool landing = false;

    //functions:
    CDH();
    void init();
    
    void flight();
    void recovery();
    
    void readBMP180();
    void readBNO055();
    void readGPS();
    void readRTC();

    void Log();
    void Transmit();
}



#endif
