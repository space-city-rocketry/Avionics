#include "CDH.h"
#include "FlightLogic.h"
#include <EEPROM.h>

#define DEBUG
#if defined DEBUG
  #define debugln(a) (Serial.println(a))
#else
  #define debugln(a)
#endif

CDH cdh;
FlightLogic prometheus;

enum FlightState {
  Standby,
  Ascent,
  Descent,
  Recovery
};

FlightState state = Standby; //Save to EEPROM 

void setup() {
  cdh.init();
  //if necessary:
  //prometheus.init(); 
  Serial.begin(115200);
  debugln("Debug ON");
}

void loop() {
  state = stateCheck();
  switch(state)
  {
    case Standby:
      Standby();
      break;
    case Ascent:
      Ascent();
      break;
    case Descent:
      Descent();
      break;
    case Recovery:
      Recovery();
      break;
  }

}

int stateCheck()
{
  //EEPROM access?
  //Make sure that previous detections were not false
}

void Standby()
{
  //Ground testing code. TBD
}

void Ascent()
{
  cdh.flight();
  if(prometheus.launchDetect())
    cdh.launch = true;
  if(prometheus.apogeeDetect())
  {
    cdh.apogee = true;
    state = Descent;
  }
}

void Descent()
{
  cdh.flight();
  if(prometheus.mainDetect())
    cdh.main = true;
  if(prometheus.landingDetect())
  {
    cdh.landing = true;
    state = Recovery; 
  }
}

void Recovery()
{
  cdh.recovery();
}

