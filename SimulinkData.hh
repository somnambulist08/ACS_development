#ifndef SIMULINK_DATA_HH
#define SIMULINK_DATA_HH

#include "GetData.hh"
#include <SD.h>
#include <list>

#define MAX_INPUT_LINE_LENGTH 16

class SimulinkFile : public GetData {
public:
	//personal
	SimulinkFile();
	bool readLine();
	bool readVals();
	void readFrame(float &t);

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
	File simulation_log;
	std::list<float> alt;
	std::list<float> acc;
	std::list<float> time_steps;
	std::list<float>::iterator alt_iter;
	std::list<float>::iterator acc_iter;
	std::list<float>::iterator time_iter;
};
#endif