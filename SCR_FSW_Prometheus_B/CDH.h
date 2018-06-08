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

#define DEBUG
#if defined DEBUG
  #define debugln(a) (Serial.println(a))
  #define debug(a) (Serial.print(a))
#else
  #define debugln(a)
  #define debug(a)
#endif

#define XBEE Serial1
#define GPS Serial2

#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

class CDH
{
  public:
    //BMP180 data variables:
    float bmpAltitude;
    float bmpPressure;
    float bmpTemp;
    float deltaH, AltOld;
    //BNO055 data variables:
    float tiltx, tilty, tiltz;
    float accelx, accely, accelz;

    //other data variables/objects:
    Adafruit_BNO055 bno = Adafruit_BNO055(55);
    Adafruit_BMP085 bmp;
    TinyGPS gps;

    //Detections
    bool launch = false;
    bool apogee = false;
    bool main = false;
    bool landing = false;

    //functions:
    CDH();
    void init();

    void standby();
    void flight();
    void recovery();

    void readBMP180();
    void readBNO055();
    void readGPS();
    void readRTC();

    void Log();
    void Transmit();

    void disp();
    void plotTilt();

    time_t getTeensy3Time(); //RTC function

    unsigned long processSyncMessage(); 


};



#endif
