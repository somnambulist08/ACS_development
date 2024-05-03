#include "ExternalSensors.hh"

ExternalSensors::ExternalSensors(){
  BMP_a = new Adafruit_BMP280((int8_t)(CSBa), &SPI);
  IMU_a = new MPU9250_WE(&SPI, (int)(NCSa), true);
}
ExternalSensors::~ExternalSensors(){
  delete BMP_a;
  delete IMU_a;
}
void ExternalSensors::startupTasks(){
  for(int i=0; i<3; i++){
    ExternalSensors::gyroOffsets[i] = 0;
    ExternalSensors::accOffsets[i] = 0;
  }

  if (!BMP_a->begin()) {
    if(Serial) Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }
    BMP_a->setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X2,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
    if(Serial) {
      Serial.print("Barometer Setup Complete, starting pressure at P = : ");
      float press = BMP_a->readPressure();
      Serial.println(String(press));
    }
    // exposeTHESENUTS = press;
    // yankTHESENUTS = pressureAlt(exposeTHESENUTS);
  int status = IMU_a->init();
  if (status < 0) {
    if(Serial) Serial.println("IMU initialization unsuccessful");
    if(Serial) Serial.println("Check IMU wiring or try cycling power");
    if(Serial) Serial.print("Status: ");
    if(Serial) Serial.println(status);
    while(1) {}
  }

//   IMU_a->autoOffsets();
  IMU_a->setMagOpMode(AK8963_PWR_DOWN);
  IMU_a->enableGyrDLPF();
  IMU_a->setGyrDLPF(MPU9250_DLPF_0);
  IMU_a->enableAccDLPF(true);
  IMU_a->setAccDLPF(MPU9250_DLPF_0);
  IMU_a->setSampleRateDivider(8);
  IMU_a->setAccRange(MPU9250_ACC_RANGE_16G);
  IMU_a->setGyrRange(MPU9250_GYRO_RANGE_2000);


  delayMicroseconds(100);
 
  //Manually Set 16G range
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(NCSa, 0);
  SPI.transfer(28); //register 28: Accel Config 1
  SPI.transfer(0x18); //16G mode
  digitalWrite(NCSa, 1);
  delayMicroseconds(100);
  SPI.endTransaction(); 

  //Manually Set 2000dps range //WARNING: Removes DLPF work //TODO: figure out what value the Fchoice_b bit should be
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(NCSa, 0);
  SPI.transfer(27); //register 27: Gyro Config
  SPI.transfer(0x18); //2000dps mode
  digitalWrite(NCSa, 1);
  delayMicroseconds(100);
  SPI.endTransaction();






  if(Serial) Serial.println("Calibrating offsets. Don't move");
  calibrateOffsets();
  if(Serial) Serial.println("Offset Calibration Complete");
  if(Serial) Serial.println("Gyro Offsets: ");
  if(Serial) Serial.print("X:");
  if(Serial) Serial.print(gyroOffsets[0], 9);
  if(Serial) Serial.print(", Y:");
  if(Serial) Serial.print(gyroOffsets[1], 9);
  if(Serial) Serial.print(", Z:");
  if(Serial) Serial.println(gyroOffsets[2], 9);

  if(Serial) Serial.println("Acc Offsets: ");
  if(Serial) Serial.print("X:");
  if(Serial) Serial.print(accOffsets[0], 9);
  if(Serial) Serial.print(", Y:");
  if(Serial) Serial.print(accOffsets[1], 9);
  if(Serial) Serial.print(", Z:");
  if(Serial) Serial.println(accOffsets[2], 9);


  



}
void ExternalSensors::readAcceleration(float &x, float &y, float &z){
    xyzFloat acc = IMU_a->getGValues();
    x = -acc.x ;/// 8.0f; //I think it's reporting in m/s2 so this is to convert to G
    y = -acc.y ;/// 8.0f; //OR maybe it is just not scaling properly and we need to divide by 8 because 
    // z = -(acc.z - 1.0f);/// 8.0f; //we went from 2G to 16G range? //ALso, there is a bais on this axis
    z = -acc.z;
    x -= accOffsets[0];
    y -= accOffsets[1];
    z -= accOffsets[2];
}
void ExternalSensors::readAltitude(float &H){
    H=pressureAlt(BMP_a->readPressure());
}

void ExternalSensors::readGyroscope(float &x, float &y, float &z){
    xyzFloat gyr = IMU_a->getGyrValues();
    x=gyr.x ;/// 8.0f; //the gyros also seem to be having a problem with the increased resolution
    y=gyr.y ;/// 8.0f;
    z=gyr.z ;/// 8.0f;  
    x -= gyroOffsets[0];
    y -= gyroOffsets[1];
    z -= gyroOffsets[2];
}
void ExternalSensors::readMagneticField(float &x, float &y, float &z){
    //placeholders so nothing breaks downstream - mag off for power
    x = -1;
    y = -1;
    z = -1; 
}
void ExternalSensors::readPressure(float &P){
  P = BMP_a->readPressure();
}
void ExternalSensors::readTemperature(float &T){
    T=BMP_a->readTemperature();
}

void ExternalSensors::calibrateOffsets(){
#ifndef STATIC_OFFSETS
  float sumGyro[3] = {0.0f,0.0f,0.0f};
  float sumAcc[3] = {0.0f,0.0f,0.0f};
  float gyro[3] = {0.0f,0.0f,0.0f};
  float acc[3] = {0.0f,0.0f,0.0f};
  
  for(int i=0; i<CALIBRATION_LOOPS; i++){
    delay(1);
    readGyroscope(gyro[0], gyro[1], gyro[2]);
    readAcceleration(acc[0], acc[1], acc[2]);
    for(int axis = 0; axis < 3; axis++){
      sumGyro[axis] += gyro[axis];
      sumAcc[axis] += acc[axis];
    }
  }

  for(int axis = 0; axis < 3; axis++){
    gyroOffsets[axis] = sumGyro[axis] / ((float)CALIBRATION_LOOPS);
    accOffsets[axis] = sumAcc[axis] / ((float)CALIBRATION_LOOPS);
  }
#else//if def STATIC_OFFSETS
  gyroOffsets[0] = -0.07457257;
  gyroOffsets[1] = -0.246435553;
  gyroOffsets[2] = 1.148041487;
  accOffsets[0] =  0.020606153;
  accOffsets[1] = 0.086461306;
  accOffsets[2] = -0.096222267;
#endif //STATIC_OFFSETS
  
}
