#ifndef FlightLogic_h
#define FlightLogic_h
#include "Arduino.h"

#define DEBUG
#if defined DEBUG
  #define debugln(a) (Serial.println(a))
  #define debug(a) (Serial.print(a))
#else
  #define debugln(a)
  #define debug(a)
#endif

class FlightLogic
{
  public:
    FlightLogic();
    bool launchDetect(int Altitude, int Acceleration);
    bool apogeeDetect(int DeltaH,int Acceleration);
    bool mainDetect(int Altitude,int Acceleration);
    bool landingDetect(int Altitude,int Acceleration);
};

#endif
