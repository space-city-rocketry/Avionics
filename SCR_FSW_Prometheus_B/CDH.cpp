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
    debugln("Unable to sync with the RTC");
    delay(1000);
  } else {
    debugln("RTC has set the system time");
  }
  if (!bmp.begin())
  {
    debugln("ERROR: Failure to initialize BMP180");
    delay(1000);
  }
  if(!bno.begin())
  {
    debugln("ERROR: Failure to initialize BNO055");
    delay(1000);
  }
  
  XBEE.begin(9600);
  
  GPS.begin(9600);
  GPS.println(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Send GPS setup command
//  GPS.println("$PMTK251,115200*1F"); //set to 115200
//  GPS.begin(115200); //switch to 115200
  
  METStart = now();
  debugln("CDH initialization complete"); 
  delay(1000);

  while(!GPS.available()){}
}

void CDH::standby()
{
  //syncGPS();
  readBMP180();
  readBNO055();
  
  //readGPS();
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
  
  for (unsigned long start = millis(); millis() - start < GPS_timeout;) {
    while (GPS.available()) {
      char c = GPS.read();
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
      {
        newData = true;
        gps.f_get_position(&GPSLat, &GPSLong, &fix_age);
        GPSAlt = gps.f_altitude();
        GPSSats = gps.satellites();
        //gps.get_datetime(&GPSDate, &cdh.data.MET, &fix_age);
      }
    }
  }
  Serial.println();
}

void CDH::syncGPS(Timer t)
{
  while(!GPS.available() | GPS_ON == false) {t.update();}
  if(GPS_ON){readGPS();}
  //GPSsync = millis();
  //debugln(GPSsync);
}

void CDH::readRTC()
{
  //RTC code
  MET = METStart - now();
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
  debugln("BMP180 Data:");
  debug("bmpTemp: \t");debug(bmpTemp);debug("\tbmpPressure: \t");debugln(bmpPressure);
  debug("bmpAltitude: \t");debug(bmpAltitude);debug("\tdeltaH: \t");debug(deltaH);
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
