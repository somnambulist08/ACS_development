#include "InternalSensors.hh"
InternalSensors::InternalSensors(){
};
void InternalSensors::startupTasks(){
  fixedIMU = sixteenIMU(Wire1);
  // onboard IMU: 16g range, 2000dps gyro
    if (!fixedIMU.begin())
    { // startup
      Serial.println("Failed to initialize IMU!");
      while (1)
      {
        digitalWrite(RED, HIGH);
      }
    }
    else
    {
      Serial.println("IMU good.");
    }
    if (!HS300x.begin())
    {
      Serial.println("Failed to initialize thermometer!");
      while (1)
      {
        digitalWrite(RED, HIGH);
      }
    }
    else
    {
      Serial.println("Thermometer good.");
    }
    if (!BARO.begin())
    {
      Serial.println("Failed to initialize pressure sensor!");
      while (1)
      {
        digitalWrite(RED, HIGH);
      }
    }
    else
    {
      Serial.println("Barometer good.");
    }
};
void InternalSensors::readAcceleration(float &x, float &y, float &z){
  if(fixedIMU.accelerationAvailable())
    fixedIMU.readAcceleration(x,y,z);
};
void InternalSensors::readAltitude(float &H){
  H = pressureAlt(BARO.readPressure()*1000.0f);
};
void InternalSensors::readGyroscope(float &x, float &y, float &z){
  if(fixedIMU.gyroscopeAvailable())
    fixedIMU.readGyroscope(x,y,z);
};
void InternalSensors::readMagneticField(float &x, float &y, float &z){
  if(fixedIMU.magneticFieldAvailable())
    fixedIMU.readMagneticField(x,y,z);
};
void InternalSensors::readPressure(float &P){
  P = BARO.readPressure()*1000.0f;
};
void InternalSensors::readTemperature(float &T){
  T = HS300x.readTemperature();
};
