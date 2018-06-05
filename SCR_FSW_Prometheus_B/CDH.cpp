#include "Arduino.h"
#include "CDH.h"

CDH::CDH()

void CDH::init()
{
  //CDH initialization
  bmp.begin()
  bno.begin()
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

