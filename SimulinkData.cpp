#include "SimulinkData.hh"


SimulinkFile::SimulinkFile(){
    
}

void SimulinkFile::loadData(const char* file) {
    Serial.println("Entering loadData()");
    fileName = file;
    dataFile = SD.open(fileName);
    if (dataFile) {
    while (dataFile.available() && dataCount < MAX_DATA_POINTS) {
        String line = dataFile.readStringUntil('\n');
        parseLine(line);
    }
    dataFile.close();
    } else {
    Serial.println("Error opening data file!");
    }
}

void SimulinkFile::parseLine(String line) {
    int firstCommaIndex = line.indexOf(',');
    int lastCommaIndex = line.lastIndexOf(',');

    if (firstCommaIndex == -1 || lastCommaIndex == -1 || firstCommaIndex == lastCommaIndex) {
    Serial.println("Invalid line format");
    return;
    }

    String altitudeStr = line.substring(0, firstCommaIndex);
    String accelerationStr = line.substring(firstCommaIndex + 1, lastCommaIndex);
    String timeStr = line.substring(lastCommaIndex + 1);

    time[dataCount] = timeStr.toFloat();
    acceleration[dataCount] = accelerationStr.toFloat();
    altitude[dataCount] = altitudeStr.toFloat();
    dataCount++;
}

float SimulinkFile::interpolate(float x[], float y[], float xKey, int size) {
  Serial.print("Entering interpolate with key=");
  Serial.println(xKey);
  if (xKey <= x[0]) return y[0];
  if (xKey >= x[size - 1]) return y[size - 1];

  for (int i = 0; i < size - 1; i++) {
    if (xKey >= x[i] && xKey <= x[i + 1]) {
      float slope = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
      return y[i] + slope * (xKey - x[i]);
    }
  }
  return 0; // Default case, should not reach here if xKey is within bounds
}

float SimulinkFile::getInterpolatedAltitude(float timeKey) {
  return interpolate(time, altitude, timeKey, dataCount);
}

float SimulinkFile::getInterpolatedAcceleration(float timeKey) {
  return interpolate(time, acceleration, timeKey, dataCount);
}

void SimulinkFile::printData() {
  for (int i = 0; i < dataCount; i++) {
    Serial.print("Time: ");
    Serial.print(time[i]);
    Serial.print("s, Acceleration: ");
    Serial.print(acceleration[i]);
    Serial.print("m/s^2, Altitude: ");
    Serial.print(altitude[i]);
    Serial.println("m");
  }
}

void SimulinkFile::startupTasks(){
  if (!SD.begin()) {
    Serial.println("SD card initialization failed!");
    return;
  }
  loadData("STATET~1.CSV"); //CONTRO~1 or STATET~1
}
void SimulinkFile::readAcceleration(float &x, float &y, float &z){

}
void SimulinkFile::readAltitude(float &H){

}

void SimulinkFile::readMagneticField(float &x, float &y, float &z){}
void SimulinkFile::readGyroscope(float &x, float &y, float &z){}
void SimulinkFile::readTemperature(float &T){}
void SimulinkFile::readPressure(float &P){}


	