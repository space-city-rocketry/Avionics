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
Timer SYNC;
bool isSynced = false;
unsigned long _start_;

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
  _start_ = now();

  Serial.begin(115200);
#ifdef DEBUG
  while (!Serial) {}
#endif

  //if necessary:
  //prometheus.init();
  //Serial.println("Testing");
  debugln("Debug ON");


  //cdh.syncGPS();
  cdh.init();
  t.every(250, FlightControl);

  sync();
}

void loop() {
  t.update();
  SYNC.update();
  //  cdh.syncGPS();
}

void sync() 
{
  unsigned long _start = millis();
  debug("GPS Execution start: "); debugln(_start);
  cdh.syncGPS(t);
  if (isSynced == false)
  {
    SYNC.every(1000, sync);
    //isSynced = true;
  }
  unsigned long gpstime = millis() - _start;
  //if (gpstime < cdh.GPS_timeout){isSynced = true;}
  debug("GPS Execution time: "); debugln(gpstime);
}

void FlightControl()
{
  //debugln("Loop");
  unsigned long _start = millis();
  debug("Execution start: "); debugln(_start);
  state = stateCheck();
  switch (state)
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
  debug("Execution time: "); debugln(millis() - _start);
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
  if (prometheus.launchDetect())
    cdh.launch = true;
  if (prometheus.apogeeDetect())
  {
    cdh.apogee = true;
    state = Descent;
  }
}

void descent()
{
  cdh.flight();
  if (prometheus.mainDetect())
    cdh.main = true;
  if (prometheus.landingDetect())
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

