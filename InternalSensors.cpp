#include "InternalSensors.hh"

/*
long accelerint(float a0, float a1, float dt) {
  return dt * (a1 + a0) / 2.0;  //central Reimann sum
}*/

class sixteenIMU : public BoschSensorClass
{
public:
  sixteenIMU(TwoWire &wire = Wire)
      : BoschSensorClass(wire){};
  virtual int readAcceleration(float &x, float &y, float &z)
  {
    int rv = BoschSensorClass::readAcceleration(x, y, z);
    x = -4.0f * x;
    y = 4.0f * y;
    z = -4.0f * z;
    return rv;
  }
  virtual int readMagneticField(float &x, float &y, float &z)
  {
    int rv = BoschSensorClass::readMagneticField(x, y, z);
    x = -x;
    z = -z;
    return rv;
  }
  virtual int readGyroscope(float &x, float &y, float &z)
  {
    int rv = BoschSensorClass::readGyroscope(x, y, z);
    y = -y;
    return rv;
  }

protected:
  virtual int8_t configure_sensor(struct bmi2_dev *dev)
  {
    int8_t rslt;
    uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};

    struct bmi2_int_pin_config int_pin_cfg;
    int_pin_cfg.pin_type = BMI2_INT1;
    int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
    int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

    struct bmi2_sens_config sens_cfg[2];
    sens_cfg[0].type = BMI2_ACCEL;
    sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_25HZ;
    sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_16G;

    sens_cfg[1].type = BMI2_GYRO;
    sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;
    sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_sensor_enable(sens_list, 2, dev);
    if (rslt != BMI2_OK)
      return rslt;

    return rslt;
  }
};
sixteenIMU sixteenIMU(Wire1);
class InternalSensors : public GetData
{
  float pressureAlt(float pressure)
  {
    return (1 - powf((pressure / 101325), 0.190284)) * 145366.45 * 0.3048;
  }
  void startupTasks()
  {
    // onboard IMU: 16g range, 2000dps gyro
    if (!sixteenIMU.begin())
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
  }
  void readAcceleration(float &x, float &y, float &z){
    sixteenIMU.readAcceleration(x,y,z);
  }
  void readMagneticField(float &x, float &y, float &z){
    sixteenIMU.readMagneticField(x,y,z);
  }
  void readGyroscope(float &x, float &y, float &z){
    sixteenIMU.readGyroscope(x,y,z);
  }
  void readTemperature(float &T){
    T = HS300x.readTemperature();
  }
  void readAltitude(float &H){
    H = pressureAlt(BARO.readPressure()*1000.0f); 
  }
};
