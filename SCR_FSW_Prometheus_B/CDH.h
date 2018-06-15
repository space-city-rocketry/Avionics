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
#include <Timer.h>
#include <SD.h>

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

#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define xbee(a) XBEE.print(a)
#define xbeeln(a) XBEE.println(a)

#define logdata(a) dataFile.print(a)
#define logdataln(a) dataFile.println(a)

enum FlightState {
      Standby,
      Ascent,
      Descent,
      Recovery
    };

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
    float accelMag;

    //RTC
    time_t NOW; //Current time in current time zone
    time_t METStart;
    time_t MET;
    int MET_sec;
    int MET_min;
    int MET_hour;

    //GPS
    TinyGPS gps;
    unsigned long GPSsync;
    int GPS_timeout = 200;
    bool GPS_ON = true;
    long GPSTime;
    float GPSLat;
    float GPSLong;
    float GPSAlt;
    int GPSSats;
    bool newData; //GPS new data check
    unsigned long fix_age; //GPS fix age check

    //Data Logger
    File dataFile; //SD file
    //const int chipselect = -1; //To turn off data logging (debugging)
    const int chipselect = BUILTIN_SDCARD; //SD card chipselect variable
    const char* fileName; //SD card file name
    bool logging;

    //Transmitter
    int packetCnt;

    //Sensors
    Adafruit_BNO055 bno = Adafruit_BNO055(55);
    Adafruit_BMP085 bmp;
    
    //States
    FlightState *cdhstate;
    String curState;

    //Detections
    bool launch = false;
    bool apogee = false;
    bool main = false;
    bool landing = false;

    //functions:
    CDH();
    void init(FlightState *state);

    void standby();
    void flight();
    void recovery();

    void readBMP180();
    void readBNO055();
    void readGPS();
    void syncGPS(elapsedMillis &TIME);
    void readRTC();

    bool Log();
    void Transmit();

    void disp();
    void plotTilt();

    void stateCheck();
};



#endif
