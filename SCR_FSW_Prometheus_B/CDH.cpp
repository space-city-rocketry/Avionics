#include "Arduino.h"
#include "CDH.h"

CDH::CDH()
{

}

void CDH::init()
{
  //CDH initialization
  // set the Time library to use Teensy 3.0's RTC to keep time

  if (timeStatus() != timeSet) {
    //Serial.println("Unable to sync with the RTC");
  } else {
    //Serial.println("RTC has set the system time");
  }
  bmp.begin();
  bno.begin();
  XBEE.begin(9600);
  GPS.begin(9600);
  GPS.println(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Send GPS setup command
  debugln("CDH initialization complete"); 
}

void CDH::standby()
{
  readBMP180();
  readBNO055();
  //disp();
  //plotTilt();
}

void CDH::flight()
{
  //flight operations

}

void CDH::recovery()
{
  //recovery operations
}

void CDH::readBMP180()
{
  bmpTemp = bmp.readTemperature();
  bmpPressure = bmp.readPressure();
  bmpAltitude = bmp.readAltitude(102438.26); //CALIBRATION NEEDED
  deltaH = bmpAltitude - AltOld;
  AltOld = bmpAltitude;
}

void CDH::readBNO055()
{
  sensors_event_t event;
  bno.getEvent(&event);
  tiltx = event.orientation.x;
  tilty = event.orientation.y;
  tiltz = event.orientation.z;
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accelx = accel.x();
  accely = accel.y();
  accelz = accel.z();

//  if (startupFlag == 1) {
//    baselineAccel = accel.z();
//    Serial.print(baselineAccel);
//  }
}

void CDH::readGPS()
{
  //GPS code
}

void CDH::readRTC()
{
  //RTC code
}

void CDH::Log()
{
  //Data logging
}

void CDH::Transmit()
{
  //Data transmission
}

void CDH::disp()
{
  debug("BMP180 Data:");
  debug("bmpTemp: \t");debug(bmpTemp);debug("\tbmpPressure: \t");debugln(bmpPressure);
  debug("bmpAltitude: \t");debug(bmpAltitude);debug("deltaH: \t");debug(deltaH);
  debugln();debugln("BNO055 Data:");
  debug("tiltx: \t");debug(tiltx);debug("\taccelx: \t");debugln(accelx);
  debug("tilty: \t");debug(tilty);debug("\taccely: \t");debugln(accely);
  debug("tiltz: \t");debug(tiltz);debug("\taccelz: \t");debugln(accelz);
}

void CDH::plotTilt()
{
  debug(tiltx);debug(" ");debug(tilty);debug(" ");debug(tiltz);debugln(" ");
}
/*  code to process time sync messages from the serial port   */


unsigned long CDH::processSyncMessage() {
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
