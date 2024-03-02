#include <Arduino.h>
/******************************************************************************
 * nova2_development.ino
 * 
 * Contains the runnable code for development, testing, and flight
 * 
 * Uncomment one of the following defines to choose which test to run
******************************************************************************/
//#define DEVELOPMENT
//#define STATE_TEST
//#define CONTROL_TEST
#define FLIGHT
//#define SENSORTEST

/*****************************************************************************
 * DEVELOPMENT
*****************************************************************************/
#ifdef DEVELOPMENT

#include "RocketRTOS.hh"
#include "SerialSpoofStepper.hh"
#include "Control.hh"
//#include "InternalSensors.hh"
#include "SDSpoofer.hh"


SDSpoofer dummySD;
SerialSpoofStepper stepper;

unsigned long oldMicros;
unsigned long oldAccel;

float accel=0;
float vel=0;
float h=0;
float ang=0;


void setup(){
  Serial.begin(115200);
  while(!Serial);

  dummySD.openFile();
  oldMicros = micros();
  oldAccel = 0;

  startRocketRTOS();
}

void stepper_RUN(){
  Serial.println("Stepper Run");
  stepper.stepOnce();
}

void stepper_IDLE(){
  Serial.println("Stepper Idle");
}

void determineState(){
  Serial.println("Determining State");
  rocketState = ROCKET_FREEFALL;
}

void sensorAndControl_FULL(){
  float x=4, y=0, z=8;
  //sensor.readAcceleration(x,y,z);

  float pressure = 80000;

  h = (1 - powf((pressure / 101325), 0.190284)) * 145366.45 * 0.3048;


  float dt = micros() - oldMicros;
  vel = (dt * (z + oldAccel) / 2.0);


  ang = getControl(100, predictAltitude(h, vel), dt);


  //stepper.setStepsTarget(stepper.microStepsFromFlapAngle(flaps));
  stepper.setStepsTarget(1000000);
}

void logging_RUN(){
  dummySD.writeLog(accel, vel, h, ang);
}

#endif //DEVELOPMENT



/*****************************************************************************
 * STATE TEST
*****************************************************************************/
#ifdef STATE_TEST
#include <mbed.h>
#include "RocketRTOS.hh"
#include "SDLogger.hh"
#include "RealStepper.hh"
#include "Control.hh"
#include "SAAM.hh"
#include "InternalSensors.hh"
#include "BZZT.hh"
#include "SimulinkData.hh"
#include "SerialSpoofStepper.hh"

#define LAUNCH_THRESHOLD_A_M_S2 10
#define LAUNCH_THRESHOLD_H_M 20

#define G_TO_M_S2 9.8f

void prvReadSensors();
void prvIntegrateAccel();
void prvSensorFusion();
void prvDoControl();

//we need a new timer because they last 30 minutes before they overflow.
//If we sit on the pad for longer than that then we don't know if we are just barely
//in the new 30 minutes or if we are going to overflow mid-flight. To avoid this,
//we just reset the timer until it is finally time to run. That way, we are garunteed
//to not overflow during launch
mbed::Timer tim;
mbed::Timer simTimer;

SimulinkFile simIn;
RealStepper stepper;
//SerialSpoofStepper stepper;
SDLogger sd;
InternalSensors sensors;

float tNow=0;
float tOld=0;
float newAcc;
float vel=0;
float oldAcc=0;
float h=0;
float oldH=0;
float ang=0;
float h_groundLevel=0;

float a_raw[3] = {0,0,0};
float m[3] = {0,0,0};
float a[3] = {0,0,0};

void setup(){
  //Serial.begin(115200);
  //while(!Serial);
  longBzzt(1); //1 long means we are in setup
  delay(1000);

  simIn.startupTasks();

  sensors.startupTasks();
  sd.openFile();
  longBzzt(2); //1 more long means we have the SD ready
  delay(1000);

  /*do{
    sensors.readAcceleration(a_raw[0], a_raw[1], a_raw[2]);
    sensors.readMagneticField(m[0], m[1], m[2]);
  }while(!zerocal(a_raw[0], a_raw[1], a_raw[2], m[0], m[1], m[2]));
  bzzt(3); // 3 short means calibration successfull
  delay(500);*/
  sensors.readAltitude(h_groundLevel);

  simTimer.start();
  tim.start();
  startRocketRTOS();
}

void determineState(){
  while(newAcc < LAUNCH_THRESHOLD_A_M_S2 && h < LAUNCH_THRESHOLD_H_M ){
    //Serial.println("PRE");
    rocketState = ROCKET_PRE;
    delay(STATE_CHECKING_DELAY_MS);
  }
  while(newAcc > 0){
    //Serial.println("LAUNCH");
    rocketState = ROCKET_LAUNCH;
    delay(STATE_CHECKING_DELAY_MS);
  }
  while(vel>0){
    //Serial.println("FREEFALL");
    rocketState = ROCKET_FREEFALL;
    delay(STATE_CHECKING_DELAY_MS);
  }
  while(1){ //once you enter recovery state, do not leave
    //Serial.println("RECOVERY");
    rocketState = ROCKET_RECOVERY;
    delay(STATE_CHECKING_DELAY_MS);
  }
}

void sensorAndControl_PRE(){
  tim.reset(); //continue to reset tim until we determine we are in launch mode
  //get data
  prvReadSensors();

  updateVars();
}
void sensorAndControl_LAUNCH(){
  //get data
  prvReadSensors();

  //Apply sensor fusion
  prvSensorFusion();


  //integrate acc to get vel
  prvIntegrateAccel();

  //update variables
  updateVars();
}
void sensorAndControl_FULL(){
  //get data
  prvReadSensors();

  //Apply sensor fusion
  prvSensorFusion();

  //integrate acc to get vel
  prvIntegrateAccel();

  //update variables
  updateVars();

  //set control value
  prvDoControl();

}


void logging_RUN(){
  //Serial.println("Entering logging_RUN");
  sd.writeLog(a_raw[0], a_raw[1], a_raw[2], a[0], a[1], a[2], m[0], m[1], m[2], ang, h, tNow);
}
void logging_CLOSE(){
  sd.closeFile();
}

void stepper_RUN(){
  stepper.stepOnce();
}
void stepper_CLOSE(){
  stepper.setStepsTarget(0);
  for(int s=0; s<100; s++){
    stepper.stepOnce();
  }
}

void buzz_PRE(){
  bzzt(1); 
}
void buzz_POST(){
  bzzt(2);
}


void prvReadSensors(){
  //Serial.println("Entering prvReadSensors");
  sensors.readAcceleration(a_raw[0], a_raw[1], a_raw[2]);
  sensors.readMagneticField(m[0], m[1], m[3]);
  float tempH=0;
  sensors.readAltitude(tempH);
  //convert H to AGL
  //Serial.print("H from Sensor:");
  //Serial.println(tempH - h_groundLevel);
  //convert A to m/s2
  //Serial.print("A from Sensors:");
  //Serial.println(a_raw[2] * G_TO_M_S2); //TODO: if sensor fusion works, then change this!

  float t = ((float)(simTimer.elapsed_time().count()))/1000000.0f;
  h = simIn.getInterpolatedAltitude(t);
  newAcc = simIn.getInterpolatedAcceleration(t);
  //Serial.print("h=");
  //Serial.println(h);
  //Serial.print("newAcc=");
  //Serial.println(newAcc);

}
void prvIntegrateAccel(){
  //Serial.println("Entering prvIntegrateAccel");
  tNow = ((float)(tim.elapsed_time().count()))/1000000.0f;
  float dt = (tNow - tOld);

  //TODO: Choose one!
  //vel += (oldAcc + newAcc)/2.0f * dt;
  vel = (h - oldH)/dt;
  //Serial.print("vel=");
  //Serial.println(vel);
}
void prvSensorFusion(){
  /*
  Quaternion q_rot = rotDiff(SAAM(a_raw,m), q_origin);
  Quaternion a_q = {.w=0, .x=a_raw[0], .y=a_raw[1], .z=a_raw[2]};
  Quaternion a_q_originFrame = hamProduct(hamProduct(q_rot, a_q),  conjugate(q_rot));
  a[0] = a_q_originFrame.x;
  a[1] = a_q_originFrame.y;
  a[2] = a_q_originFrame.z;
  newAcc = a[2];*/
}
void prvDoControl(){
  ang = getControl(getDesired(tNow), predictAltitude(h,vel), tNow-tOld);
  stepper.setStepsTarget(microStepsFromFlapAngle(ang));
}
void updateVars(){
  //Serial.println("Entering updateVars");
  oldAcc = newAcc; 
  tOld = tNow;
  oldH = h;
}





#endif //STATE_TEST

/*****************************************************************************
 * CONTROL TEST
*****************************************************************************/
#ifdef CONTROL_TEST

#include "RocketRTOS.hh"
//#include "SDLogger.hh"
#include "SDSpoofer.hh"
#include "SimulinkData.hh"
#include "RealStepper.hh"
#include "Control.hh"
//#include "SerialSpoofStepper.hh"
#include <mbed.h>

mbed::Timer tim;

SimulinkFile simIn;
RealStepper stepper;
//SerialSpoofStepper stepper;
SDSpoofer out;

float accX=0, accY=0, accZ=0;
float vel=275.6;
float h=0;
float ang=0;
float oldAccel=0;
float tNow=0;
float tLast=0;

void setup(){
  Serial.begin(115200);
  while(!Serial);
  
  delay(1000);
  Serial.println("Attempting Startup Tasks Now");
  simIn.startupTasks();
  simIn.printData();
  Serial.println("Finished, Prepare to Enter the Scheduler!");
  delay(1000);

  tim.start();
  startRocketRTOS();
}


void sensorAndControl_FULL(){
  //get data
  tNow = ( (float)(tim.elapsed_time().count()) ) / 1000000.0f;
  Serial.print("tNow: ");
  Serial.println(tNow);
  accZ = simIn.getInterpolatedAcceleration(tNow);
  Serial.print("accZ: ");
  Serial.println(accZ);
  h = simIn.getInterpolatedAltitude(tNow);
  Serial.print("h: ");
  Serial.println(h);

  //integrate
  float dt = tNow - tLast;
  vel += (accZ + oldAccel)/2.0f * dt;
  Serial.print("dt: ");
  Serial.println(dt);
  Serial.print("AccInt: ");
  Serial.println((float)((accZ + oldAccel)/2.0f * dt));
  Serial.print("vel: ");
  Serial.println(vel);
  
  //set control value
  ang = getControl(getDesired(tNow), predictAltitude(h,vel), dt);
  Serial.print("ang: ");
  Serial.println(ang);
  Serial.print("H Desired: ");
  Serial.println(getDesired(tNow));
  Serial.print("H Predicted: ");
  Serial.println(predictAltitude(h,vel));

  stepper.setStepsTarget(microStepsFromFlapAngle(ang));

  //set old values
  oldAccel = accZ;
  tLast = tNow;
}

void stepper_RUN(){
  stepper.stepOnce();
}

void logging_RUN(){
  out.writeLog(accZ, vel, h, ang);
}



#endif //CONTROL_TEST


/*****************************************************************************
 * FLIGHT
*****************************************************************************/
#ifdef FLIGHT
#include <mbed.h>
#include "RocketRTOS.hh"
#include "SDLogger.hh"
#include "RealStepper.hh"
#include "Control.hh"
#include "SAAM.hh"
#include "InternalSensors.hh"
#include "BZZT.hh"
#include "SimulinkData.hh"

#define LAUNCH_THRESHOLD_A_M_S2 10
#define LAUNCH_THRESHOLD_H_M 20

#define G_TO_M_S2 9.8f

void prvReadSensors();
void prvIntegrateAccel();
void prvSensorFusion();
void prvDoControl();

//we need a new timer because they last 30 minutes before they overflow.
//If we sit on the pad for longer than that then we don't know if we are just barely
//in the new 30 minutes or if we are going to overflow mid-flight. To avoid this,
//we just reset the timer until it is finally time to run. That way, we are garunteed
//to not overflow during launch
mbed::Timer tim;

SimulinkFile simIn;
RealStepper stepper;
SDLogger sd;
InternalSensors sensors;

float tNow=0;
float tOld=0;
float newAcc;
float vel=0;
float oldAcc=0;
float h=0;
float oldH=0;
float ang=0;
float h_groundLevel=0;

float a_raw[3] = {0,0,0};
float m[3] = {0,0,0};
float a[3] = {0,0,0};

void setup(){
  //Serial.begin(115200);
  //while(!Serial);
  longBzzt(1); //1 long means we are in setup
  delay(1000);

  sensors.startupTasks();
  sd.openFile();
  longBzzt(2); //1 more long means we have the SD ready
  delay(1000);

  /*do{
    sensors.readAcceleration(a_raw[0], a_raw[1], a_raw[2]);
    sensors.readMagneticField(m[0], m[1], m[2]);
  }while(!zerocal(a_raw[0], a_raw[1], a_raw[2], m[0], m[1], m[2]));
  bzzt(3); // 3 short means calibration successfull
  delay(500);*/
  sensors.readAltitude(h_groundLevel);

  tim.start();
  startRocketRTOS();
}

void determineState(){
  while(newAcc < LAUNCH_THRESHOLD_A_M_S2 && h < LAUNCH_THRESHOLD_H_M ){
    //Serial.println("PRE");
    rocketState = ROCKET_PRE;
    delay(STATE_CHECKING_DELAY_MS);
  }
  while(newAcc > 0){
    //Serial.println("LAUNCH");
    rocketState = ROCKET_LAUNCH;
    delay(STATE_CHECKING_DELAY_MS);
  }
  while(vel>0){
    //Serial.println("FREEFALL");
    rocketState = ROCKET_FREEFALL;
    delay(STATE_CHECKING_DELAY_MS);
  }
  while(1){ //once you enter recovery state, do not leave
    //Serial.println("RECOVERY");
    rocketState = ROCKET_RECOVERY;
    delay(STATE_CHECKING_DELAY_MS);
  }
}

void sensorAndControl_PRE(){
  tim.reset(); //continue to reset tim until we determine we are in launch mode
  //get data
  prvReadSensors();

  updateVars();
}
void sensorAndControl_LAUNCH(){
  //get data
  prvReadSensors();

  //Apply sensor fusion
  prvSensorFusion();


  //integrate acc to get vel
  prvIntegrateAccel();

  //update variables
  updateVars();
}
void sensorAndControl_FULL(){
  //get data
  prvReadSensors();

  //Apply sensor fusion
  prvSensorFusion();

  //integrate acc to get vel
  prvIntegrateAccel();

  //update variables
  updateVars();

  //set control value
  prvDoControl();

}


void logging_RUN(){
  //Serial.println("Entering logging_RUN");
  sd.writeLog(a_raw[0], a_raw[1], a_raw[2], a[0], a[1], a[2], m[0], m[1], m[2], ang, h, tNow);
}
void logging_CLOSE(){
  sd.closeFile();
}

void stepper_RUN(){
  stepper.stepOnce();
}
void stepper_CLOSE(){
  stepper.setStepsTarget(0);
  for(int s=0; s<100; s++){
    stepper.stepOnce();
  }
}

void buzz_PRE(){
  bzzt(1); 
}
void buzz_POST(){
  bzzt(2);
}


void prvReadSensors(){
  //Serial.println("Entering prvReadSensors");
  sensors.readAcceleration(a_raw[0], a_raw[1], a_raw[2]);
  sensors.readMagneticField(m[0], m[1], m[3]);
  float tempH=0;
  sensors.readAltitude(tempH);
  //convert H to AGL
  //Serial.print("H from Sensor:");
  //Serial.println(tempH - h_groundLevel);
  //convert A to m/s2
  //Serial.print("A from Sensors:");
  //Serial.println(a_raw[2] * G_TO_M_S2); //TODO: if sensor fusion works, then change this!

}
void prvIntegrateAccel(){
  //Serial.println("Entering prvIntegrateAccel");
  tNow = ((float)(tim.elapsed_time().count()))/1000000.0f;
  float dt = (tNow - tOld);

  //TODO: Choose one!
  //vel += (oldAcc + newAcc)/2.0f * dt;
  vel = (h - oldH)/dt;
  //Serial.print("vel=");
  //Serial.println(vel);
}
void prvSensorFusion(){
  /*
  Quaternion q_rot = rotDiff(SAAM(a_raw,m), q_origin);
  Quaternion a_q = {.w=0, .x=a_raw[0], .y=a_raw[1], .z=a_raw[2]};
  Quaternion a_q_originFrame = hamProduct(hamProduct(q_rot, a_q),  conjugate(q_rot));
  a[0] = a_q_originFrame.x;
  a[1] = a_q_originFrame.y;
  a[2] = a_q_originFrame.z;
  newAcc = a[2];*/
}
void prvDoControl(){
  ang = getControl(getDesired(tNow), predictAltitude(h,vel), tNow-tOld);
  stepper.setStepsTarget(microStepsFromFlapAngle(ang));
}
void updateVars(){
  //Serial.println("Entering updateVars");
  oldAcc = newAcc; 
  tOld = tNow;
  oldH = h;
}


#endif //FLIGHT



/*****************************************************************************
 * SENSORTEST
*****************************************************************************/
#ifdef SENSORTEST

#include "RocketRTOS.hh"
//#include "SerialSpoofStepper.hh"
#include "Control.hh"
#include "InternalSensors.hh"
#include "SDSpoofer.hh"
#include "SDLogger.hh"

SDSpoofer dummySD;
//SDLogger sd; //seeing if this is causing a runtime error//jonse
//SerialSpoofStepper stepper;
InternalSensors intSensors; //patmer: declare internal sensors object


float accel=0;
float vel=0;
float h=0;
float ang=0;

float magfield_data_x = 0; // variable that holds the measured magnetometer x-data
float magfield_data_y = 0; // variable that holds the measured magnetometer y-data
float magfield_data_z = 0; // variable that holds the measured magnetometer z-data
float gyro_data_x = 0; // variable that holds the measured gyroscope x-data
float gyro_data_y = 0; // variable that holds the measured gyroscope x-data
float gyro_data_z = 0; // variable that holds the measured gyroscope x-data
float altitude_data = 0; // variable that holds the measured altitude
float accelerometer_data_x = 0;
float accelerometer_data_y = 0;
float accelerometer_data_z = 0;

float pressure_data = -1.0f;
float temperature_data = -1.0f;



void setup(){
  intSensors.startupTasks();
  Serial.begin(115200);
  while(!Serial);

  dummySD.openFile();
  //sd.openFile();

  startRocketRTOS();
}

/* patmer Orientation & altitude gathering functions*/
void getOrientationAltitude(){ //patmer get orientation & altitude
  Serial.println("Getting orientation and altitude");
  


  //you missed one :) //jonse
  intSensors.readAcceleration(accelerometer_data_x, accelerometer_data_y, accelerometer_data_z);
  intSensors.readPressure(pressure_data);
  intSensors.readTemperature(temperature_data);
  intSensors.readMagneticField(magfield_data_x, magfield_data_y, magfield_data_z);
  intSensors.readGyroscope(gyro_data_x, gyro_data_y, gyro_data_z);
  intSensors.readAltitude(altitude_data);


  //fill the variables that get logged //jonse
  h = altitude_data;
  accel = accelerometer_data_z;

  Serial.println("Temperature:");
  Serial.println(temperature_data);
  Serial.println("Pressure:");
  Serial.println(pressure_data);
  Serial.print("acc_x: ");
  Serial.print(accelerometer_data_x);
  Serial.print(" acc_y: ");
  Serial.print(accelerometer_data_y);
  Serial.print(" acc_z: ");
  Serial.println(accelerometer_data_z);

  Serial.print("gyro_x: ");
  Serial.print(gyro_data_x);
  Serial.print(" gyro_y: ");
  Serial.print(gyro_data_y);
  Serial.print(" gyro_z: ");
  Serial.println(gyro_data_z);
  
  Serial.print("mag_x: ");
  Serial.print(magfield_data_x);
  Serial.print(" mag_y: ");
  Serial.print(magfield_data_y);
  Serial.print(" mag_z: ");
  Serial.println(magfield_data_z);

}

void displayOrientationAltitude_SDSpoof(){ //patmer Display results over serial with SDSpoofer
  Serial.println("Displaying data to SDSpoofer");



  dummySD.writeLog(accel, vel, h, ang);
}

void displayOrientationAltitude_SD(){ //patmer Save results to a real SD card
  //Serial.println("Displaying data to real SD");

  //sd.writeLog(accel, vel, h, ang);
}
/* patmer Orientation & altitude gathering functions END*/



/* Scheduler functions*/
void sensorAndControl_FULL(){
  getOrientationAltitude();
}

void logging_RUN(){
  //displayOrientationAltitude_SDSpoof();
  displayOrientationAltitude_SD();
}
/* Scheduler functions END*/

#endif //SENSORTEST

