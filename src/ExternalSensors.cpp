#include "ExternalSensors.hh"



Adafruit_BMP280 BMP_a(CSBa);
MPU9250_WE IMU_a = MPU9250_WE(&SPI, NCSa, true);
ExternalSensors::ExternalSensors(){
  for(int i=0; i<3; i++){
    gyroOffsets[i] = 0;
    accOffsets[1] = 0;
  }
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

//   IMU_a.autoOffsets();
  IMU_a.enableGyrDLPF();
  IMU_a.enableAccDLPF(true);
  IMU_a.setAccDLPF(MPU9250_DLPF_0);
  IMU_a.setAccRange(MPU9250_ACC_RANGE_16G);
  IMU_a.setGyrRange(MPU9250_GYRO_RANGE_2000);
  IMU_a.setMagOpMode(AK8963_PWR_DOWN);
  IMU_a.setGyrDLPF(MPU9250_DLPF_0);
  IMU_a.setSampleRateDivider(8);

  if(Serial) Serial.println("Calibrating offsets. Don't move");
  calibrateOffsets();
  if(Serial) Serial.println("Offset Calibration Complete Offsets: ");
  if(Serial) Serial.print("X:");
  if(Serial) Serial.print(gyroOffsets[0]);
  if(Serial) Serial.print(", Y:");
  if(Serial) Serial.print(gyroOffsets[1]);
  if(Serial) Serial.print(", Z:");
  if(Serial) Serial.println(gyroOffsets[2]);

};
void ExternalSensors::readAcceleration(float &x, float &y, float &z){
    xyzFloat acc = IMU_a.getGValues();
    x = -acc.x ;/// -8.0f; //I think it's reporting in m/s2 so this is to convert to G
    y = -acc.y ;/// -8.0f; //OR maybe it is just not scaling properly and we need to divide by 8 because 
    z = -(acc.z - 1.0f) ;/// -8.0f; //we went from 2G to 16G range? //ALso, there is a bais on this axis
    x -= accOffsets[0];
    y -= accOffsets[1];
    z -= accOffsets[2];
};
void ExternalSensors::readAltitude(float &H){
    H = pressureAlt(BMP_a.readPressure());
};
void ExternalSensors::readGyroscope(float &x, float &y, float &z){
    xyzFloat gyr = IMU_a.getGyrValues();
    x=gyr.x ;/// 8.0f; //the gyros also seem to be having a problem with the increased resolution
    y=gyr.y ;/// 8.0f;
    z=gyr.z ;/// 8.0f;  
    x -= gyroOffsets[0];
    y -= gyroOffsets[1];
    z -= gyroOffsets[2];
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

void ExternalSensors::calibrateOffsets(){
  float sumGyro[3] = {0.0f,0.0f,0.0f};
  // float sumAcc[3] = {0.0f,0.0f,0.0f};
  float gyro[3] = {0.0f,0.0f,0.0f};
  // float acc[3] = {0.0f,0.0f,0.0f};
  
  for(int i=0; i<CALIBRATION_LOOPS; i++){
    delay(1);
    readGyroscope(gyro[0], gyro[1], gyro[2]);
    // readAcceleration(acc[0], acc[1], acc[2]);
    for(int axis = 0; axis < 3; axis++){
      sumGyro[axis] += gyro[axis];
      // sumAcc[axis] += acc[axis];
    }
  }

  for(int axis = 0; axis < 3; axis++){
    gyroOffsets[axis] = sumGyro[axis] / ((float)CALIBRATION_LOOPS);
    // accOffsets[axis] = sumGyro[axis] / 100.0f;
  }
  
}
