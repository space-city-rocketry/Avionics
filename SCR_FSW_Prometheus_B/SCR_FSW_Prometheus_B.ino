#include "CDH.h"
#include "FlightLogic.h"

CDH cdh;
FlightLogic prometheus;

enum FlightState {
  Standby,
  Ascent,
  Descent,
  Recovery
};

FlightState state = Standby;

void setup() {
  

}

void loop() {
  

}
