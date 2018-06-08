#include "CDH.h"
#include "FlightLogic.h"
#include <EEPROM.h>
#include <Timer.h>

#define DEBUG
#if defined DEBUG
  #define debugln(a) (Serial.println(a))
  #define debug(a) (Serial.print(a))
#else
  #define debugln(a)
  #define debug(a)
#endif



CDH cdh;
FlightLogic prometheus;

Timer t;

enum FlightState {
  Standby,
  Ascent,
  Descent,
  Recovery
};

FlightState state = Standby; //Save to EEPROM 

time_t getTeensy3Time() //RTC function
{
  return Teensy3Clock.get();
}

void setup() {
  setSyncProvider(getTeensy3Time);
  
  Serial.begin(115200);
  cdh.init();
  //if necessary:
  //prometheus.init(); 
  //Serial.println("Testing");
  debugln("Debug ON");

  t.every(10, FlightControl);
}

void loop() {
  t.update();
  
}

void FlightControl()
{
  //debugln("Loop");
  state = stateCheck();
  switch(state)
  {
    case Standby:
      standby();
      //debugln("standby");
      break;
    case Ascent:
      ascent();
      debugln("ascent");
      break;
    case Descent:
      descent();
      debugln("descent");
      break;
    case Recovery:
      recovery();
      debugln("recovery");
      break;
  }
}

FlightState stateCheck()
{
  //EEPROM access?
  //Make sure that previous detections were not false
}

void standby()
{
  //Ground testing code. TBD
  cdh.standby();
  
}

void ascent()
{
  cdh.flight();
  if(prometheus.launchDetect(cdh.accelx))
    cdh.launch = true;
  if(prometheus.apogeeDetect())
  {
    cdh.apogee = true;
    state = Descent;
  }
}

void descent()
{
  cdh.flight();
  if(prometheus.mainDetect())
    cdh.main = true;
  if(prometheus.landingDetect())
  {
    cdh.landing = true;
    state = Recovery; 
    //set slower timer rate
  }
}

void recovery()
{
  cdh.recovery();
}

