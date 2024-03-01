#include "Arduino_BMI270_BMM150.h"  //IMU
class sixteenIMU : public BoschSensorClass {
public:
    sixteenIMU();
  sixteenIMU(TwoWire &wire);
  virtual int readAcceleration(float &x, float &y, float &z);
  virtual int readMagneticField(float &x, float &y, float &z);
  virtual int readGyroscope(float &x, float &y, float &z);
protected:
  virtual int8_t configure_sensor(struct bmi2_dev *dev);
};