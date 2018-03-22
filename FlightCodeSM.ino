#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP085.h>
#include "SoftwareSerial.h"

char a[] = "<";
char b[] = ">,";
int packetno = 1;
int pressure, altitude, temp, tiltx, tilty, tiltz;
int incomingByte = 0;
int hOld = 0;
int StandbyFlag,AscentFlag,DescentFlag,RecoveryFlag,xaccel,yaccel,zaccel;
int int AccelCalibration; 
int A1,A2,A3, v;
int BaroStorage[3];
int GPSStorage[3]; 
int AccelStorage[3];
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP085 bmp;


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

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}}
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  
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

StandbyFlag = 1;
AscentFlag = 0;
DescentFlag = 0;
RecoveryFlag = 0;

int AccelFlag =0;
}

}


void loop(void)
{



  while(StandbyFlag = 1){  //Standby
   sensors_event_t event;
   bno.getEvent(&event);
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
   tiltx = event.orientation.x;
   tilty = event.orientation.y;
   tiltz = event.orientation.z;   
   xaccel = euler.x();
   yaccel = euler.y();
   zaccel = euler.z(); 
  delay(BNO055_SAMPLERATE_DELAY_MS);

    temp = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude();
    deltaH = altitude - hOld;
    hOld = bmp.readAltitude();
    //packno, altitude, pressure, temp, accelx, accely, accelz, tiltx, tilty, tiltz
  Serial.println(a + packetno + b + a + altitude + b + a + pressure + b + a + temp + b + a + xaccel + b + a + yaccel + b + a + zaccel + b + a + tiltx + b + a + tilty + b + a + tiltz + b);
  packetno = packetno + 1;

if AccelFlag = 0{
AccelFlag =1;
AccelCalibrationx = xaccel;
AccelCalibrationy = yaccel;
AccelCalibrationz = zaccel;


}

  if(deltaH > 0.5 && zaccel > AccelCalibrationz){
    StandbyFlag = 0;
    AscentFlag = 1;
    Serial.println(AscentFlag);

  }
  
  
  }
  
  while(AscentFlag = 1){  //Ascent
   sensors_event_t event;
   bno.getEvent(&event);
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
   xaccel = euler.x();
   yaccel = euler.y();
   zaccel = euler.z(); 
   tiltx = event.orientation.x;
   tilty = event.orientation.y;
   tiltz = event.orientation.z;   
  
   delay(BNO055_SAMPLERATE_DELAY_MS);

    temp = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude();
    deltaH = altitude - hOld;
    hOld = bmp.readAltitude();
  Serial.println(a + packetno + b + a + altitude + b + a + pressure + b + a + temp + b + a + xaccel + b + a + yaccel + b + a + zaccel + b + a + tiltx + b + a + tilty + b + a + tiltz + b);
  packetno = packetno + 1;

  A1 = 1/(deltaH+1); //Baro
  A2 = 9.81/accel // IMU
  //A3 = 1/(deltaH+1); //GPS

  v = sqrt(A1^2+A2^2+A3^2) * 100;
  if( v =! 300 || v=! 325 || v=! 275){}
  if(v == 300 || v == 325 || v == 275){
    AscentFlag = 0;
    DescentFlag = 1;
    int ApogeeH = altitude;
    Serial.println(DescentFlag);
    Serial.println(ApogeeH);
    }
  }

  
  while(DescentFlag = 1){  //Descent
     sensors_event_t event;
   bno.getEvent(&event);
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
   xaccel = euler.x();
   yaccel = euler.y();
   zaccel = euler.z();    
   tiltx = event.orientation.x;
   tilty = event.orientation.y;
   tiltz = event.orientation.z;   
  
   delay(BNO055_SAMPLERATE_DELAY_MS);

    temp = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude();
    deltaH = altitude - hOld;
    hOld = bmp.readAltitude();
  Serial.println(a + packetno + b + a + altitude + b + a + pressure + b + a + temp + b + a + xaccel + b + a + yaccel + b + a + zaccel + b + a + tiltx + b + a + tilty + b + a + tiltz + b);
  packetno = packetno + 1;

  if(altitude == h1){
    digitalWrite(pin1,HIGH);
    delay(100);
    digitalWrite(pin1,LOW);
    int DrogueDeploymentFlag =1;
    Serial.println(DrogueDeploymentFlag);
    
  }
   if(altitude == h2){
    digitalWrite(pin2,HIGH);
    delay(100);
    digitalWrite(pin2,LOW);
    int MainDeploymentFlag =1;
    Serial.println(MainDeploymentFlag);\
    DescentFlag=0;
    StandbyFlag=1;
    
  }
  while(StandbyFlag = 1){  //Recovery

   sensors_event_t event;
   bno.getEvent(&event);
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
   xaccel = euler.x();
   yaccel = euler.y();
   zaccel = euler.z(); 
   tiltx = event.orientation.x;
   tilty = event.orientation.y;
   tiltz = event.orientation.z;   
  
   delay(BNO055_SAMPLERATE_DELAY_MS);

    temp = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude();
    deltaH = altitude - hOld;
    hOld = bmp.readAltitude();
  Serial.println(a + packetno + b + a + altitude + b + a + pressure + b + a + temp + b + a + xaccel + b + a + yaccel + b + a + zaccel + b + a + tiltx + b + a + tilty + b + a + tiltz + b);
    packetno = packetno + 1;
    
    delay(10000);
    
  
  }
  


  if (Serial.available() > 0) {incomingByte = Serial.read();}
  if (incomingByte != 0){  
    digitalWrite(6, HIGH);
    delay(500);
    digitalWrite(6, LOW);
  }
 
}
