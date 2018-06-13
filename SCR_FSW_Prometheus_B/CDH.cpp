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
  if (!bno.begin())
  {
    debugln("ERROR: Failure to initialize BNO055");
    delay(1000);
  }

  XBEE.begin(9600);

  GPS.begin(9600);
  GPS.println(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Send GPS setup command
  //  GPS.println("$PMTK251,115200*1F"); //set to 115200
  //  GPS.begin(115200); //switch to 115200

  NOW = now();
  METStart = NOW;
  debugln("CDH initialization complete");
  delay(1000);

  while (!GPS.available()) {}
}

void CDH::standby()
{
  readBMP180();
  readBNO055();
  readRTC();
  disp();
  Transmit();
}

void CDH::flight()
{
  //flight operations
  standby();

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

void CDH::syncGPS(elapsedMillis &TIME)
{
  while (!GPS.available()) {}
  Serial.print("GPS start; TIME = "); Serial.println(TIME);
  TIME = 0;
  readGPS();
}

void CDH::readRTC()
{
  //RTC code
  NOW = now();
  MET = NOW - METStart;
  MET_sec = second(MET);
  MET_min = minute(MET);
  MET_hour = hour(MET);
}

void CDH::Log()
{
  //Data logging
}

void CDH::Transmit()
{
  //Data transmission
  xbeeln("Transmission Test");
  xbee("Current Mission Elapsed Time (MET):\t");
  xbee(MET_hour);xbee(":");xbee(MET_min);xbee(":");xbeeln(MET_sec);
}

void CDH::disp()
{
  debugln("BMP180 Data:");
  debug("bmpTemp: \t"); debug(bmpTemp); debug("\tbmpPressure: \t"); debugln(bmpPressure);
  debug("bmpAltitude: \t"); debug(bmpAltitude); debug("\tdeltaH: \t"); debug(deltaH);
  debugln(); debugln("BNO055 Data:");
  debug("tiltx: \t"); debug(tiltx); debug("\taccelx: \t"); debugln(accelx);
  debug("tilty: \t"); debug(tilty); debug("\taccely: \t"); debugln(accely);
  debug("tiltz: \t"); debug(tiltz); debug("\taccelz: \t"); debugln(accelz);
  debugln();debugln("Time Data:");
  debug("MET:\t");
  debug(MET_hour);debug(":");debug(MET_min);debug(":");debugln(MET_sec);
  debug("CST:\t");
  debug(hour(NOW));debug(":");debug(minute(NOW));debug(":");debugln(second(NOW));
  debugln();
}

void CDH::plotTilt()
{
  debug(tiltx); debug(" "); debug(tilty); debug(" "); debug(tiltz); debugln(" ");
}

/*  code to process time sync messages from the serial port   */



