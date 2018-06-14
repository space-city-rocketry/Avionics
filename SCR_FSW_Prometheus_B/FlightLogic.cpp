#include "Arduino.h"
#include "FlightLogic.h"

FlightLogic::FlightLogic(){}


bool FlightLogic::launchDetect(int Altitude, int Acceleration)
{
  bool status = false;
  int n = 0;
  int m = 0;

  //checking Altitude
  if(Altitude > 0 && Altitude < 5)
  {
    n = 1;
  }
  
  //checking acceleration
  if(Acceleration > 5 && Acceleration < 10)
  {
    m = 1;
  }

  //for status to be true both condition must be satisfied 
  if(m == 1 && n == 1 )
  {
    status = true;
  }

  return status;
}

bool FlightLogic::apogeeDetect(int DeltaH,int Acceleration)
{
  bool status = false;
  int n = 0;
  int m = 0;

  //checking Delta H to be 0 m with 0.1 margin of error 
  if(DeltaH > -0.1 && DeltaH < 0.1)
  {
    n = 1;
  }
  
  //checking acceleration to be -9.8 m/s^2 with 1 margin of error
  if(Acceleration > -10.8 && Acceleration < -8.8)
  {
    m = 1;
  }

  //for status to be true both condition must be satisfied 
  if(m == 1 && n ==1)
  {
    status = true;
  }

  return status;
  
}

bool FlightLogic::mainDetect(int Altitude,int Acceleration) 
{
    bool status = false;
  int n = 0;
  int m = 0;

  //checking Altitude
  if(Altitude == 1000)
  {
    n = 1;
  }
  
  //checking acceleration 
  if(Acceleration > -0.4 && Acceleration < 0.4)
  {
    m = 1;
  }

  //for status to be true both condition must be satisfied 
  if(m == 1 && n ==1)
  {
    status = true;
  }

  return status;
}

bool FlightLogic::landingDetect(int Altitude,int Acceleration)
{
  bool status = false;
  int n = 0;
  int m = 0;

  //checking Altitude
  if(Altitude > 0 && Altitude < 20)
  {
    n = 1;
  }
  
  //checking acceleration 
  if(Acceleration == 0)
  {
    m = 1;
  }

  //for status to be true both condition must be satisfied 
  if(m == 1 && n == 1)
  {
    status = true;
  }

  return status;
}


