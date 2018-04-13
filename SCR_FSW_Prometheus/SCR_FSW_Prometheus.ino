/* Avionics Flight Software Alpha 0.1 - RevA
   Authors: Samar Mathur & Michael Greer

   The Flight Avionics subsystem in the Space City Rocketry competition team is
   responsible for executing the major operations of the Recovery phase of the
   IREC mission CONOPS.

   This software package controls the secondary system in the Avionics package,
   which is a custom electronics package with an Arduino-type microcontroller
   acting as the main flight computer. The system consists of multiple sensors
   together with a transmitter and data logger that monitors, saves, and transmits
   mission data and uses it to make operational decisions during flight.

   Current Capabilities list:

*/


//Library includes
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP085.h>
#include "SoftwareSerial.h"
#include <Adafruit_GPS.h>
#include <SPI.h>
#include "Timer.h"
#include <SD.h>


Timer t;

//Preprocessor Macro declarations
#define BNO055_SAMPLERATE_DELAY_MS (100)

const int chipSelect = 53;
enum FSWState {
  kStandby,
  kAscent,
  kDescent,
  kRecovery

};

FSWState state = kStandby;
  
int packetno = 1;
float pressure, altitude, gpsalt, temp, tiltx, tilty, tiltz, xaccel, yaccel, zaccel, h1, h2, deltaH, baselineAccel;
int incomingByte = 0;
float heightOld = 0;
int StandbyFlag, AscentFlag, DescentFlag, RecoveryFlag, buzzpin;
int startupFlag = 1;
int EjectionFlag[1];
int i = 0;
int AccelCalibration;
int time1,time2;
int stateno = 1;
int ARM = 2; //Arming pin set
int EVENT_1_PIN = 3; //Pin set for Event 1, drogue deployment
int EVENT_2_PIN = 4; //Pin set for Event 2, main deployment




//int BMPCheck;
//int IMUCheck;
//int GPSCheck;
//int TotalCheck;
float BaroStorage[3];
float GPSStorage[3];
float AccelStorage[3];
//int deltaH;

SoftwareSerial mySerial(11, 10);
Adafruit_GPS GPS(&Serial1);


Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP085 bmp;

#define GPSECHO  true

//Flag to indicate if an interrupt was detected
bool intDetected = false;

void imuRetrieve(void) {
  delay(BNO055_SAMPLERATE_DELAY_MS);
  sensors_event_t event;
  bno.getEvent(&event);
  tiltx = event.orientation.x;
  tilty = event.orientation.y;
  tiltz = event.orientation.z;
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  xaccel = accel.x();
  yaccel = accel.y();
  zaccel = accel.z();

  if (startupFlag == 1){
     baselineAccel = accel.z();
     Serial.print(baselineAccel);
  }

}

void  bmpRetrieve(void) {
  temp = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude(102438.26);
  deltaH = altitude - heightOld;
  heightOld = bmp.readAltitude();
}

void GPSRetrieve(void) {
// <<<<<<< Moving-code-to-functions
//     // read data from the GPS in the 'main loop'
//     char c = GPS.read();
//     // if you want to debug, this is a good time to do it!
  
// =======
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);


// >>>>>>> testing
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
// <<<<<<< Moving-code-to-functions
  
//     if (!GPS.parse(GPS.lastNMEA())){   // this also sets the newNMEAreceived() flag to false
//       return;  // we can fail to parse a sentence in which case we should just wait for another
//   }
// =======
    if (!GPS.parse(GPS.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false
      Serial.println("Parse Unsuccessful");
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
  }
  gpsalt = GPS.altitude;
//>>>>>>> testing

  
 //int gpsalt = GPS.altitude;
  }
}

/*void apogeeDetect(void) {

BMPCheck = 1 / (deltaH + 1); //Baro
IMUCheck = 9.81 / accel // IMU
GPSCheck = 1/(deltaH+1); //GPS

   TotalCheck = sqrt(BMPCheck ^ 2 + IMUCheck ^ 2 + GPSCheck ^ 2) * 100;
  if ( TotalCheck = ! 300 || TotalCheck = ! 325 || TotalCheck = ! 275) {}
  if (TotalCheck == 300 || TotalCheck == 325 || TotalCheck == 275) {
  
   state = kDescent; //Change state to Descent
   int ApogeeH = altitude;
 }
}*/

/*void transmit(void) {
  TransmitSerial.print("<");
  TransmitSerial.print(packetno); TransmitSerial.print(",");
  TransmitSerial.print(altitude); TransmitSerial.print(",");
  TransmitSerial.print(pressure); TransmitSerial.print(",");
  TransmitSerial.print(temp); TransmitSerial.print(",");
  TransmitSerial.print(xaccel); TransmitSerial.print(",");
  TransmitSerial.print(yaccel); TransmitSerial.print(",");
  TransmitSerial.print(zaccel); TransmitSerial.print(",");
  TransmitSerial.print(tiltx); TransmitSerial.print(",");
  TransmitSerial.print(tilty); TransmitSerial.print(",");
  TransmitSerial.print(tilty); TransmitSerial.print(",");
  TransmitSerial.print(gpsalt); TransmitSerial.print(",");
  TransmitSerial.print(state); TransmitSerial.print(",");
  TransmitSerial.println(">");
}*/


void printing(void) {
  Serial.print("<");
  Serial.print(packetno); Serial.print(",");
  Serial.print(altitude); Serial.print(",");
  Serial.print(pressure); Serial.print(",");
  Serial.print(temp); Serial.print(",");
  Serial.print(xaccel); Serial.print(",");
  Serial.print(yaccel); Serial.print(",");
  Serial.print(zaccel); Serial.print(",");
  Serial.print(tiltx); Serial.print(",");
  Serial.print(tilty); Serial.print(",");
  Serial.print(tiltz); Serial.print(",");
  Serial.println(stateno);Serial.print(",");
  Serial.println(">");
}

void ejection(void) {
  if (altitude == h1) {
    digitalWrite(EVENT_1_PIN, HIGH);
    delay(100);
    digitalWrite(EVENT_1_PIN, LOW);
    int DrogueDeploymentFlag = 1;
    Serial.println(DrogueDeploymentFlag); //make clear debug messages

  }
  if (altitude == h2) {
    digitalWrite(EVENT_2_PIN, HIGH);
    delay(100);
    digitalWrite(EVENT_2_PIN, LOW);
    int MainDeploymentFlag = 1;
    Serial.println(MainDeploymentFlag);


    state = kRecovery; //Change state to Recovery

  }
}


void datalog(void) {

  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    dataFile.print("\n<");
    dataFile.print(packetno); dataFile.print(",");
    dataFile.print(altitude); dataFile.print(",");
    dataFile.print(pressure); dataFile.print(",");
    dataFile.print(temp); dataFile.print(",");
    dataFile.print(xaccel); dataFile.print(",");
    dataFile.print(yaccel); dataFile.print(",");
    dataFile.print(zaccel); dataFile.print(",");
    dataFile.print(tiltx); dataFile.print(",");
    dataFile.print(tilty); dataFile.print(",");
    dataFile.print(tiltz); dataFile.print(",");
    dataFile.print(GPS.minute + 1);
    dataFile.close();
  }
}

void setup(void) {
  Serial.begin(9600);

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");



  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  /* Initialise the sensor */


  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);



  pinMode(13, OUTPUT);

  /* Display some basic information on this sensor */

  /* Optional: Display current status */

  bno.setExtCrystalUse(true);
  Serial.println("Begin Transmission");

  //Setup XBEE line (Hardware Serial 2)
//  TransmitSerial.begin(9600);
//  TransmitSerial.println("Begin Transmission");

  //If using third arming transistor, uncomment the ARM pin declaration
  pinMode(ARM, OUTPUT);
  digitalWrite(ARM, LOW);
  pinMode(EVENT_1_PIN, OUTPUT);
  digitalWrite(EVENT_1_PIN, LOW);
  pinMode(EVENT_2_PIN, OUTPUT);
  digitalWrite(EVENT_2_PIN, LOW);

  pinMode(buzzpin, OUTPUT);
  digitalWrite(buzzpin, LOW);

  int AccelFlag = 0;
  t.every(500, MainFunction);

}


/*void buzzer(void) {
  if (startupFlag = 1) {
    for (int k = 1; k < 10; k++) {
      digitalWrite(buzzpin, HIGH); //no time spacing?
      delay(100);
      digitalWrite(buzzpin, LOW);
      
    }
  }
  for (int k = 1; k < 10; k++) {
    digitalWrite(buzzpin, HIGH);
    delay(100);
    digitalWrite(buzzpin, LOW);
  }

}*/
void loop(void){t.update();}


void detectAscent(void){
   
  //look at accelerometer data and delta h data, if accelerometer is above the baseline, then change state to ascent. 
  if((zaccel - baselineAccel > .5) && (deltaH > 0)){
    state = kAscent;
    stateno = 2;
  }
  
}
void MainFunction(void){
  if (startupFlag = 1) {
//    buzzer();
   imuRetrieve();
   bmpRetrieve();
    startupFlag = 0;
  }

  switch (state) {
    case kStandby: //Standby state code block
      time1 = millis();
      imuRetrieve();
      bmpRetrieve();
      detectAscent();
      GPSRetrieve();
      //transmit();
      printing();
      datalog();
      packetno = packetno + 1;
      time2 = millis()-time1;
      Serial.print(time2);
      break;
    case kAscent: //Ascent state code block
  //    imuRetrieve();
 //     bmpRetrieve();
//      transmit();
      // apogeeDetect();
      break;
    case kDescent: //Descent state code block
 //     imuRetrieve();
 //     bmpRetrieve();
 //     transmit();
      //      ejection();
      break;
    case kRecovery: //Recovery state code block
 //     imuRetrieve();
 //     bmpRetrieve();
 //     transmit();
      delay(10000);
      break;
  }

}
//}
