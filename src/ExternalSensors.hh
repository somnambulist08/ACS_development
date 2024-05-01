/******************************************************************************
* ExternalSensors.hh
*
* Migrating onto the GY-91 10 DOF board
* MPU-9250 AMG + BMP280 Barometer
*
* 04/01/2024 - Created file
******************************************************************************/


#ifndef EXTERNAL_SENSORS_HH
#define EXTERNAL_SENSORS_HH

#include <math.h>
#include "SPI.h"
#include "Adafruit_BMP280.h" //Barometer
#include "MPU9250_WE.h"
#include "GetData.hh"


#define nice 69

//sensor pins
//GY-91a
#define CSBa 20 //bmp280
#define NCSa 21 //MPU9250
//GY-91b
#define CSBb 40 //bmp280
#define NCSb 41 //MPU9250
#define CALIBRATION_LOOPS 10000
#define STATIC_OFFSETS

#define pressureAlt(pressure) ((1.0f - powf((((float)pressure) / 101325.0f), 0.190284f)) * 145366.45f * 0.3048f)


class ExternalSensors : public GetData
{
public:
  // personal
  ExternalSensors();
  // inherited
  virtual void startupTasks() override;
  virtual void readAcceleration(float &x, float &y, float &z) override;
  virtual void readMagneticField(float &x, float &y, float &z) override;
  virtual void readGyroscope(float &x, float &y, float &z) override;
  virtual void readTemperature(float &T) override;
  virtual void readAltitude(float &H) override;
  virtual void readPressure(float &P) override;
  void calibrateOffsets();

  ~ExternalSensors();

private:
  float gyroOffsets[3];
  float accOffsets[3];

  Adafruit_BMP280 *BMP_a;
  MPU9250_WE *IMU_a;
};

#endif