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

#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

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

    time_t getTeensy3Time() //RTC function
    {
      return Teensy3Clock.get();
    }

    /*  code to process time sync messages from the serial port   */


    unsigned long processSyncMessage() {
      unsigned long pctime = 0L;
      const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

      if (Serial.find(TIME_HEADER)) {
        pctime = Serial.parseInt();
        return pctime;
        if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
          pctime = 0L; // return 0 to indicate that the time is not valid
        }
      }
      return pctime;
    }
}



#endif
