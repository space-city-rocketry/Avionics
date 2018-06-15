#include "Arduino.h"
#include "CDH.h"



CDH::CDH()
{

}

void CDH::init(FlightState *state)
{
  //CDH initialization
  cdhstate = state;
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

  fileName = "T_0615.csv"; //8 character limit
  if (chipselect >= 0) {
    logging = true;
    if (!SD.begin(chipselect)) {
      debugln("ERROR: Failure to initialize SD Logger");
      logging = false;
    }
  } else {
    logging = false;
    debugln("Data logging OFF");
  }
  if (logging) {
    //Print file headers:
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile)
    {
      logdataln("MET,\tAltitude,\tPressure,\tTemperature,\tDeltaH,\tAcceleration X,\tY,\tZ,\tOrientation X,\tY,\tZ,\tGPS Latitude,\tGPS Longitude,\tSoftware State");
      dataFile.close();
    } else {
      debugln("ERROR: Unknown error when opening data file");
      logging = false;
    }
  }

  XBEE.begin(9600);
  packetCnt = 0;

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
  elapsedMillis execTime;
  readBMP180();
  readBNO055();
  readRTC();
  disp();
  Transmit();
  Log();
  debug("Execution Time:\t");debugln(execTime);
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
  accelMag = sqrt(sq(accelx) + sq(accely) + sq(accelz));

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

bool CDH::Log()
{
  //Data logging
  if (logging) {
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile)
    {
      logdata(MET_hour); logdata(":"); logdata(MET_min); logdata(":"); logdata(MET_sec); logdata(",");
      logdata(bmpAltitude); logdata(","); logdata(bmpPressure); logdata(","); logdata(bmpTemp); logdata(","); logdata(deltaH); logdata(",");
      logdata(accelx); logdata(","); logdata(accely); logdata(","); logdata(accelz); logdata(",");
      logdata(tiltx); logdata(","); logdata(tilty); logdata(","); logdata(tiltz); logdata(",");
      logdata(GPSLat); logdata(","); logdata(GPSLong); logdata(",");
      stateCheck();
      logdata(curState);
      logdataln();
      dataFile.close();
      return true;
    } else {
      return false;
    }
  }
}

void CDH::Transmit()
{ //Add checksum for error checking?
  //Data transmission
  packetCnt++;
  xbee("$XBEE,"); xbee(packetCnt); xbee(","); //Packet start
  xbee(MET_hour); xbee(":"); xbee(MET_min); xbee(":"); xbee(MET_sec); xbee(","); //MET
  xbee(bmpAltitude); xbee(","); xbee(bmpPressure); xbee(","); xbee(bmpTemp); xbee(","); xbee(deltaH); xbee(","); //BMP180
  xbee(accelx); xbee(","); xbee(accely); xbee(","); xbee(accelz); xbee(","); //BNO055 Accel
  xbee(tiltx); xbee(","); xbee(tilty); xbee(","); xbee(tiltz); xbee(","); //BNO055 Tilt
  xbee(GPSLat); xbee(","); xbee(GPSLong); xbee(","); //GPS
  stateCheck();
  xbee(curState); //State
  xbeeln();
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
  debug("Acceleration Magnitude: \t");debugln(accelMag);
  debugln(); debugln("Time Data:");
  debug("MET:\t");
  debug(MET_hour); debug(":"); debug(MET_min); debug(":"); debugln(MET_sec);
  debug("CST:\t");
  debug(hour(NOW)); debug(":"); debug(minute(NOW)); debug(":"); debugln(second(NOW));
  stateCheck();
  debug("Current State: "); debugln(curState);
  debugln();
}

void CDH::plotTilt()
{
  debug(tiltx); debug(" "); debug(tilty); debug(" "); debug(tiltz); debugln(" ");
}

void CDH::stateCheck()
{
  if (*cdhstate == Standby) {
    curState = "Standby";
  }
  if (*cdhstate == Ascent) {
    curState = "Ascent";
  }
  if (*cdhstate == Descent) {
    curState = "Descent";
  }
  if (*cdhstate == Recovery) {
    curState = "Recovery";
  }
}

