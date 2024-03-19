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

class InternalSensors: public GetData{
public:
  //personal
  InternalSensors();
  
  //inherited
  virtual void startupTasks() override;
  virtual void readAcceleration(float &x, float &y, float &z) override;
	virtual void readMagneticField(float &x, float &y, float &z) override;
	virtual void readGyroscope(float &x, float &y, float &z) override;
	virtual void readTemperature(float &T) override;
	virtual void readAltitude(float &H) override;
	virtual void readPressure(float &P) override;
private:
  //personal
  sixteenIMU fixedIMU;
};

#endif  //INTERNAL_SENSORS_HH
