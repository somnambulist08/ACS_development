#ifndef SIMULINK_DATA_HH
#define SIMULINK_DATA_HH

#include "GetData.hh"
class SimulinkFile : public GetData {
public:
void startupTasks();
	void readFrame(float &t);
	void readAcceleration(float &x, float &y, float &z);
	void readMagneticField(float &x, float &y, float &z);
	void readGyroscope(float &x, float &y, float &z);
	void readTemperature(float &T);
	void readAltitude(float &H);
	void readPressure(float &P);
};
#endif