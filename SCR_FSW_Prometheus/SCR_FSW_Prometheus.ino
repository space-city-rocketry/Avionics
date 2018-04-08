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
//Preprocessor Macro declarations
#define BNO055_SAMPLERATE_DELAY_MS (100)
#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;
enum FSWState {
  kStandby,
  kAscent,
  kDescent,
  kRecovery

};

FSWState state = kStandby;
//Initialize software state in standby mode

//REMOVE LATER
//#define STANDBY 1   //Macro for standby state
//#define ASCENT 2    //Macro for ascent state
//#define DESCENT 3   //Macro for descent state
//#define RECOVERY 4  //Macro for recovery state

//Global variable initializations
//uint8_t swState = STANDBY; //Initialize software state in standby mode
int packetno = 1;
float pressure, altitude, temp, tiltx, tilty, tiltz,xaccel, yaccel, zaccel, h1;
int incomingByte = 0;
float heightOld = 0;
int StandbyFlag, AscentFlag, DescentFlag, RecoveryFlag, buzzpin;
int EjectionFlag[1];
int i = 0; 
int AccelCalibration;
int pinNo = 8; //Set pin numbers for ejection charges
//int BMPCheck;
//int IMUCheck;
//int GPSCheck;
//int TotalCheck;
float BaroStorage[3];
float GPSStorage[3];
float AccelStorage[3];
//int deltaH;

SoftwareSerial mySerial(2, 3); // RX, TX
SoftwareSerial GPSSerial(5, 4);

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP085 bmp;
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

//Flag to indicate if an interrupt was detected
bool intDetected = false;

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void imuRetrieve(void) {
  delay(BNO055_SAMPLERATE_DELAY_MS);
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); //wrong vector
  xaccel = euler.x();
  yaccel = euler.y();
  zaccel = euler.z();
  tiltx = event.orientation.x;
  tilty = event.orientation.y;
  tiltz = event.orientation.z;
}

void  bmpRetrieve(void) {
  temp = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude();
  //deltaH = altitude - heightOld;
  heightOld = bmp.readAltitude();
}

void GPSRetrieve(void){
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
}

//void apogeeDetect(void) {

  //BMPCheck = 1 / (deltaH + 1); //Baro
  //IMUCheck = 9.81 / accel // IMU
             //GPSCheck = 1/(deltaH+1); //GPS

          //   TotalCheck = sqrt(BMPCheck ^ 2 + IMUCheck ^ 2 + GPSCheck ^ 2) * 100;
/*  if ( TotalCheck = ! 300 || TotalCheck = ! 325 || TotalCheck = ! 275) {}
 if (TotalCheck == 300 || TotalCheck == 325 || TotalCheck == 275) {
    /* Remove Later
      AscentFlag = 0;
      DescentFlag = 1;
    */
 /*   state = kDescent; //Change state to Descent
    int ApogeeH = altitude;
    Serial.println(DescentFlag);
    Serial.println(ApogeeH); */
 // }
//}

void transmit(void) {
  mySerial.print("<");
  mySerial.print(packetno); mySerial.print(",");
  mySerial.print(altitude); mySerial.print(",");
  mySerial.print(pressure); mySerial.print(",");
  mySerial.print(temp); mySerial.print(",");
  mySerial.print(xaccel); mySerial.print(",");
  mySerial.print(yaccel); mySerial.print(",");
  mySerial.print(zaccel); mySerial.print(",");
  mySerial.print(tiltx); mySerial.print(",");
  mySerial.print(tilty); mySerial.print(",");
  mySerial.print(tilty); mySerial.print(",");
  mySerial.print(GPS.minute + 1);
  mySerial.println(">");
  packetno = packetno + 1;
}


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
  Serial.print(tiltz);
  Serial.print(GPS.minute + 1);
  Serial.println(">");
}

void ejection(void) {
  if (altitude == h1) {
    digitalWrite(pin1, HIGH);
    delay(100);
    digitalWrite(pin1, LOW);
    int DrogueDeploymentFlag = 1;
    Serial.println(DrogueDeploymentFlag);

  }
  if (altitude == h2) {
    digitalWrite(pin2, HIGH);
    delay(100);
    digitalWrite(pin2, LOW);
    int MainDeploymentFlag = 1;
    Serial.println(MainDeploymentFlag);

    
    state = kRecovery; //Change state to Recovery

  }
} 


void datalog(void){

  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
  dataFile.print("<");
  dataFile.print(packetno); dataFile.print(",");
  dataFile.print(altitude); dataFile.print(",");
  dataFile.print(pressure); dataFile.print(",");
  dataFile.print(temp); dataFile.print(",");
  dataFile.print(xaccel); dataFile.print(",");
  dataFile.print(yaccel); dataFile.print(",");
  dataFile.print(zaccel); dataFile.print(",");
  dataFile.print(tiltx); dataFile.print(",");
  dataFile.print(tilty); dataFile.print(",");
  dataFile.print(tiltz);
  dataFile.print(GPS.minute+1);

  dataFile.close();
  }
}

void setup(void){
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

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
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

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
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);
  Serial.println("Begin Transmission");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Begin Transmission");
  
  pinMode(pinNo, OUTPUT);
  digitalWrite(pinNo, LOW);

  pinMode(buzzpin, OUTPUT);
  digitalWrite(buzzpin, LOW);

  int AccelFlag = 0;
}


void buzzer(void){
  if (startupFlag = 1){
    for(int k ==1, k<10, k++){
    digitalWrite(buzzpin,HIGH);
    digitalWrite(buzzpin,LOW);
    }
  }
    for(int k ==1, k<10, k++){
    digitalWrite(buzzpin,HIGH);
    digitalWrite(buzzpin,LOW);
    }
  
}
void loop(void)
{
 if(startupFlag=1){
  buzzer();
  startupFlag = 0;
 }
  
  switch (state) {
    case kStandby: //Standby state code block
      imuRetrieve();
      bmpRetrieve();
      transmit();
      printing();
      datalog();
      
      break;
    case kAscent: //Ascent state code block
      imuRetrieve();
      bmpRetrieve();
      transmit();
     // apogeeDetect();
      break;
    case kDescent: //Descent state code block
      imuRetrieve();
      bmpRetrieve();
      transmit();
//      ejection();
      break;
    case kRecovery: //Recovery state code block
      imuRetrieve();
      bmpRetrieve();
      transmit();
      delay(10000);
      break;
  }

  if (Serial.available() > 0) {
    incomingByte = Serial.read();
  }
  if (incomingByte != 0) {
    digitalWrite(6, HIGH);
    delay(500);
    digitalWrite(6, LOW);
  }


}
//}
