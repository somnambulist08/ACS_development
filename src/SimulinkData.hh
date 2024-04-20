#ifndef SIMULINK_DATA_HH
#define SIMULINK_DATA_HH

#include "GetData.hh"
#include <SD.h>

#define MAX_INPUT_LINE_LENGTH 20
#define MAX_DATA_POINTS 12400

const int chipSelect = BUILTIN_SDCARD;//Tells Teensy to use the on-board SD

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
	virtual void startupTasks() override;
	virtual void readAcceleration(float &x, float &y, float &z) override;
	virtual void readMagneticField(float &x, float &y, float &z) override;
	virtual void readGyroscope(float &x, float &y, float &z) override;
	virtual void readTemperature(float &T) override;
	virtual void readAltitude(float &H) override;
	virtual void readPressure(float &P) override;

	//personal
	void startupTasks(const char* file);

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
	
};

#endif