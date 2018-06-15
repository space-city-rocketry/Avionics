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

//STANDBY TESTING
enum TEST {
  test_standby,
  test_ascent,
  test_descent,
  test_recovery
};
TEST test_state = test_standby;

//Data rate for ascent state
#define ASCENT_RATE 125 //8Hz Data Rate
//Data rate for recovery rate
#define REC_RATE 5000 //0.2Hz Data Rate

#define TIME_HEADER  "T"   // Header tag for serial time sync message

CDH cdh;
FlightLogic prometheus;

Timer t_ascent;
Timer t_recovery;
bool isSynced = false;
unsigned long _start_;
elapsedMillis TIME;

//enum FlightState {
//  Standby,
//  Ascent,
//  Descent,
//  Recovery
//};

FlightState state = Standby; //Save to EEPROM

time_t getTeensy3Time() //RTC function -> replace with GPS sync later
{
  return Teensy3Clock.get();
}

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


  cdh.init(&state);
  t_ascent.every(ASCENT_RATE, ascent);//Function call?
  t_recovery.every(REC_RATE, recovery);
  cdh.syncGPS(TIME); //Not a good place for this?
}

void loop() {
  unsigned long _start = millis();
  //debug("Execution start: "); debugln(_start);
  state = stateCheck();
  switch (state)
  {
    case Standby:
      standby();
      break;
    case Ascent:
      t_ascent.update();
      break;
    case Descent:
      descent();
      break;
    case Recovery:
      t_recovery.update();
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
  inputCheck();
  switch (test_state)
  {
    case test_standby:
      //Standby mode
      Serial.println("Type in any of the following commands to change the software state:\n");
      Serial.println("ascent - Turns on ascent testing mode");
      Serial.println("descent - Turns on descent testing mode");
      Serial.println("recovery - Turns on recovery testing mode\n");
      delay(5000);
      break;
    case test_ascent:
      //Ascent Prototype
      t_ascent.update();
      break;
    case test_descent:
      //Descent Prototype
      if (TIME == 250 )
      {
        debug("1a. TIME = "); debugln(TIME);
        cdh.standby();
        debug("1b. TIME = "); debugln(TIME);
      }
      if (TIME == 500) //Add error checking code
      {
        debug("2. TIME = "); debugln(TIME);
        delay(50);
      }
      if (TIME == 750)
      {
        debug("3a. TIME = "); debugln(TIME);
        cdh.standby();
        debug("3b. TIME = "); debugln(TIME);
      }
      if (TIME > 800)
      {
        debug("4. TIME = "); debugln(TIME);
        cdh.syncGPS(TIME);
        debug("Post GPS; TIME = "); debugln(TIME);
      }
      break;
    case test_recovery:
      //Recovery Prototype
      t_recovery.update();
      break;
  }
}

void ascent()
{
  cdh.flight();
  debug("Elapsed Time:\t"); debugln(TIME);
  debugln();
  //  if (prometheus.launchDetect())
  //    cdh.launch = true;
  //  if (prometheus.apogeeDetect())
  //  {
  //    cdh.apogee = true;
  //    state = Descent;
  //  }
}

void descent()
{
  if (TIME == 250 )
  {
    debug("1a. TIME = "); debugln(TIME);
    cdh.flight();
    debug("1b. TIME = "); debugln(TIME);
  }
  if (TIME == 500)
  {
    debug("2. TIME = "); debugln(TIME);
    delay(50);
  }
  if (TIME == 750)
  {
    debug("3a. TIME = "); debugln(TIME);
    cdh.flight();
    debug("3b. TIME = "); debugln(TIME);
  }
  if (TIME > 800)
  {
    debug("4. TIME = "); debugln(TIME);
    cdh.syncGPS(TIME);
    debug("Post GPS; TIME = "); debugln(TIME);
  }
  //  if (prometheus.mainDetect())
  //    cdh.main = true;
  //  if (prometheus.landingDetect())
  //  {
  //    cdh.landing = true;
  //    state = Recovery;
  //    //set slower timer rate
  //  }
}

void recovery()
{
  cdh.syncGPS(TIME);
}

void inputCheck()
{
  if (Serial.available() > 0) {
    String input = Serial.readString();
    Serial.print("String received: "); Serial.println(input);
    if (input == "ascent")
    {
      Serial.println("\nAscent Testing Mode ON\n");
      delay(1000);
      test_state = test_ascent;
    }
    if (input == "descent")
    {
      Serial.println("\nDescent Testing Mode ON\n");
      delay(1000);
      test_state = test_descent;
    }
    if (input == "recovery")
    {
      Serial.println("\nRecovery Testing Mode ON\n");
      delay(1000);
      test_state = test_recovery;
    }
    if (input == "q" | input == "exit" | input == "stop")
    {
      Serial.println("\nReturning to Standby State\n");
      delay(1000);
      test_state = test_standby;
    }
    if (input == "help" | input == "HELP")
    {
      Serial.println("Type in any of the following commands to change the software state:");
      Serial.println(" ");
      Serial.println("ascent - Turns on ascent testing mode");
      Serial.println("descent - Turns on descent testing mode");
      Serial.println("recovery - Turns on recovery testing mode");
      Serial.println("\nReturning to program in 10 seconds...\n");
      delay(10000);
    }
  }
}

