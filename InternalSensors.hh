/******************************************************************************
* InternalSensors.hh
*
* includes the classes and functions related to making the internal
* sensors do their job
*
* 02/15/2024 - Created file
******************************************************************************/


#ifndef INTERNAL_SENSORS_HH
#define INTERNAL_SENSORS_HH

#define RED 22
#define BLUE 23
#define GREEN 24
#define pressureAlt(pressure) ((1 - powf((pressure / 101325), 0.190284)) * 145366.45 * 0.3048)

#include <Arduino_HS300x.h>         //temp and humidity
#include <Arduino_LPS22HB.h>        //pressure
#include <math.h>
#include "GetData.hh"
#include "sixteenIMU.hh"
#include "SAAM.hh"

class InternalSensors: public GetData{
public:
  //personal
  InternalSensors();
  
  //inherited
  void startupTasks();
  void readAcceleration(float &x, float &y, float &z);
	void readMagneticField(float &x, float &y, float &z);
	void readGyroscope(float &x, float &y, float &z);
	void readTemperature(float &T);
	void readAltitude(float &H);
	void readPressure(float &P);
private:
  //personal
  sixteenIMU fixedIMU;
};

#endif  //INTERNAL_SENSORS_HH
