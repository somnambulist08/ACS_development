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
#define CSBa (int8_t)20 //bmp280
#define NCSa (uint8_t)21 //MPU9250
//GY-91b
#define CSBb (int8_t)40 //bmp280
#define NCSb (uint8_t)41 //MPU9250
//SPI
#define MOSI 11
#define MISO 12
#define SCK 13

#define pressureAlt(pressure) ((1 - powf((pressure / 101325), 0.190284)) * 145366.45 * 0.3048)

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
    private:
    //Adafruit_BMP280 BMP_a;
    //Adafruit_BMP280 BMP_b(CSB_b);
};

#endif