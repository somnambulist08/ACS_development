#include "ExternalSensors.hh"



Adafruit_BMP280 BMP_a(CSBa);
MPU9250_WE IMU_a = MPU9250_WE(&SPI, NCSa, true);
ExternalSensors::ExternalSensors(){
};
void ExternalSensors::startupTasks(){
    //start barometer in SPI

    if (!BMP_a.begin()) {
    if(Serial) Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }
    BMP_a.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
    if(Serial) Serial.println("Barometer Setup Complete");


  int status = IMU_a.init();
  if (status < 0) {
    if(Serial) Serial.println("IMU initialization unsuccessful");
    if(Serial) Serial.println("Check IMU wiring or try cycling power");
    if(Serial) Serial.print("Status: ");
    if(Serial) Serial.println(status);
    while(1) {}
  }
  IMU_a.setAccRange(MPU9250_ACC_RANGE_16G);
  IMU_a.setGyrRange(MPU9250_GYRO_RANGE_2000);
  IMU_a.setMagOpMode(AK8963_PWR_DOWN);
  IMU_a.enableGyrDLPF();
  IMU_a.setGyrDLPF(MPU9250_DLPF_6);
  IMU_a.setSampleRateDivider(5);
  IMU_a.enableAccDLPF(true);
  IMU_a.setAccDLPF(MPU9250_DLPF_6);


  IMU_a.autoOffsets();
};
void ExternalSensors::readAcceleration(float &x, float &y, float &z){
    xyzFloat acc = IMU_a.getGValues();
    x=acc.x;
    y=acc.y;
    z=acc.z;
};
void ExternalSensors::readAltitude(float &H){
    H = pressureAlt(BMP_a.readPressure());
};
void ExternalSensors::readGyroscope(float &x, float &y, float &z){
    xyzFloat gyr = IMU_a.getGyrValues();
    x=gyr.x;
    y=gyr.y;
    z=gyr.z;    
};
void ExternalSensors::readMagneticField(float &x, float &y, float &z){
    //placeholders so nothing breaks downstream - mag off for power
    x = -1;
    y = -1;
    z = -1; 
};
void ExternalSensors::readPressure(float &P){
    P = BMP_a.readPressure();
};
void ExternalSensors::readTemperature(float &T){
    T=BMP_a.readTemperature();
};
