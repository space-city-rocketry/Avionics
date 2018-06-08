#include "Arduino.h"
#include "CDH.h"

CDH::CDH()

void CDH::init()
{
  //CDH initialization
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
  bmp.begin();
  bno.begin();
  XBEE.begin(9600);
  GPS.begin(9600);
  GPS.println(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Send GPS setup command
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
  //BMP180 code
}

void CDH::readBNO055()
{
  //BNO055 code
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

void CDH: Transmit()
{
  //Data transmission
}

