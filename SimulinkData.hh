#ifndef SIMULINK_DATA_HH
#define SIMULINK_DATA_HH

#include "GetData.hh"
#include <SD.h>
#include <list>

#define MAX_INPUT_LINE_LENGTH 20
#define MAX_DATA_POINTS 6200


class SimulinkFile : public GetData {
public:
	//personal
	SimulinkFile();
	/*
	bool readLine(File &f, char* line, size_t maxLen);
	bool readVals(float* alt, float* acc, float* t);
	void readFrame(float &t);
	*/

	//inherited
	void startupTasks();
	void readAcceleration(float &x, float &y, float &z);
	void readMagneticField(float &x, float &y, float &z);
	void readGyroscope(float &x, float &y, float &z);
	void readTemperature(float &T);
	void readAltitude(float &H);
	void readPressure(float &P);

	//Mr GPT
	SimulinkFile(const char* file);
    void loadData(const char* file);
    void parseLine(String line);
    float interpolate(float x[], float y[], float xKey, int size);
    float getInterpolatedAltitude(float timeKey);
    float getInterpolatedAcceleration(float timeKey);
    void printData();

private:
	//personal 
	File dataFile;
    const char* fileName;

    float time[MAX_DATA_POINTS];
    float acceleration[MAX_DATA_POINTS];
    float altitude[MAX_DATA_POINTS];
    int dataCount = 0;
	
	/*File simulation_log;
	std::list<float> alt;
	std::list<float> acc;
	std::list<float> time_steps;
	std::list<float>::iterator alt_iter;
	std::list<float>::iterator acc_iter;
	std::list<float>::iterator time_iter;*/
};

#ifndef DataLogger_h
#define DataLogger_h

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

class DataLogger {
  private:
    

  public:

};

#endif

#endif