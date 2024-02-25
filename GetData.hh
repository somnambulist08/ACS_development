/****************************************************************************
* GetData.hh
* Abstract class for reading flight parameters from 
* "somewhere" - files, simulink, sensors...
****************************************************************************/
#ifndef GET_DATA_HH
#define GET_DATA_HH
#include <ArduinoAPI.h>
#include <Wire.h>

class GetData {
public:
	virtual void startupTasks() = 0;
	virtual void readAcceleration(float &x, float &y, float &z) = 0;
	virtual void readMagneticField(float &x, float &y, float &z) = 0;
	virtual void readGyroscope(float &x, float &y, float &z) = 0;
	virtual void readTemperature(float &T) = 0;
	virtual void readAltitude(float &H) = 0;
};

#endif

