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
//#define FLIGHT
//#define SENSORTEST
#define TEENSY_4_1_TESTING

/*****************************************************************************
 * DEVELOPMENT
*****************************************************************************/
#ifdef DEVELOPMENT

#include "QuickSilver.hh"
#include "InternalSensors.hh"
#include "Filter.hh"
#include "mbed.h"
#include "rtos.h"
#include "SDLogger.hh"

#define BETA 0.05f
#define DELAY_MS 100

//Gyro calibration bias. These change with temperature
#define GYRO_BIAS_X 0.138
#define GYRO_BIAS_Y 0.122
#define GYRO_BIAS_Z -0.226

#define ACC_BIAS_X -0.0012
#define ACC_BIAS_Y 0.0015
#define ACC_BIAS_Z 0.0040


#define FILTER_CUTOFF_FREQUENCY 5.0f
#define FILTER_DT_S 0.01f
#define GRYO_FILTER_PERIOD_MS 10 //100 Hz

QuickSilver attitude;
InternalSensors sensors;
pt1Filter gyroFilters[3];
SDLogger sd;

//mbed::Ticker tick;
//rtos::Thread gyroThread(osPriorityAboveNormal, 1024);
//void prvFilterGyro();

//void prvAvgGyro();

float tOld = 0.0f;
float tNow = 0.0f;

float acc[3] = {0.0f, 0.0f, 0.0f};
float gyro[3] = {0.0f, 0.0f, 0.0f};

float sum[3] = {0,0,0};
float avg[3] = {0,0,0};
unsigned int sumCount = 0;

void setup(){
  Serial.begin(115200);
  while(!Serial);

  sd.openFile();

  gyroFilters[0].init(FILTER_CUTOFF_FREQUENCY, FILTER_DT_S);
  gyroFilters[1].init(FILTER_CUTOFF_FREQUENCY, FILTER_DT_S);
  gyroFilters[2].init(FILTER_CUTOFF_FREQUENCY, FILTER_DT_S);
  sensors.startupTasks();

  //prvAvgGyro();

  attitude.initialize(BETA);
  tNow = ((float)(micros()))/1000000.0f;
  //tick.attach(&prvFilterGyro, GRYO_FILTER_PERIOD);
  //gyroThread.start(&prvFilterGyro);
}

void loop(){

  sensors.readAcceleration(acc[0], acc[1], acc[2]);
  sumCount++;
  for(int j=0; j<3; j++){
    sum[j] += acc[j];
    avg[j] = sum[j]/((float)sumCount);
  }
  Serial.print("Avg: ");
  Serial.print(avg[0], 4);
  Serial.print(", ");
  Serial.print(avg[1], 4);
  Serial.print(", ");
  Serial.println(avg[2], 4);

  sd.writeLog(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  delay(DELAY_MS);

 

  /*
  delay(DELAY_MS);
  sensors.readAcceleration(acc[0], acc[1], acc[2]);
  sensors.readGyroscope(gyro[0], gyro[1], gyro[2]);
  gyro[0] -= GYRO_BIAS_X;
  gyro[1] -= GYRO_BIAS_Y;
  gyro[2] -= GYRO_BIAS_Z;
  gyro[0] = gyroFilters[0].apply(gyro[0]);
  gyro[1] = gyroFilters[1].apply(gyro[1]);
  gyro[2] = gyroFilters[2].apply(gyro[2]);
  // Serial.print("Acc: ");
  // Serial.print(acc[0]);
  // Serial.print(", ");
  // Serial.print(acc[1]);
  // Serial.print(", ");
  // Serial.print(acc[2]);
  // Serial.println();
  // Serial.print("Gyro: ");
  // Serial.print(gyro[0]);
  // Serial.print(", ");
  // Serial.print(gyro[1]);
  // Serial.print(", ");
  // Serial.print(gyro[2]);
  // Serial.println();
  
  tOld = tNow;
  tNow = ((float)(micros()))/1000000.0f;
  float dt = tNow - tOld;
  
  attitude.update_estimate(acc,gyro,dt);
  // float *grav = attitude.getGravityVector();
  // Serial.print("Grav: ");
  // Serial.print(grav[0]);
  // Serial.print(", ");
  // Serial.print(grav[1]);
  // Serial.print(", ");
  // Serial.print(grav[2]);
  // Serial.println();
  // Serial.println();

  float vert = attitude.vertical_acceleration_from_acc(acc);
  Serial.print("Vert: ");
  Serial.println(vert);
  
  
  // Serial.print("dt: ");
  // Serial.println(dt, 6);
  // Serial.print("Hz: ");
  // Serial.println(1.0/dt);
  */
  
}


//causes hard fault???
//probably because reading the sensors is I2C and blocking and otherwise not ISR safe
/*
void prvFilterGyro(){
  sensors.readGyroscope(gyro[0], gyro[1], gyro[2]);
  gyro[0] -= GYRO_BIAS_X;
  gyro[1] -= GYRO_BIAS_Y;
  gyro[2] -= GYRO_BIAS_Z;
  gyro[0] = gyroFilters[0].apply(gyro[0]);
  gyro[1] = gyroFilters[1].apply(gyro[1]);
  gyro[2] = gyroFilters[2].apply(gyro[2]);
  Serial.print("Gyro: ");
  Serial.print(gyro[0]);
  Serial.print(", ");
  Serial.print(gyro[1]);
  Serial.print(", ");
  Serial.print(gyro[2]);
  Serial.println();

  Serial.println(micros());

  delay(GRYO_FILTER_PERIOD_MS);
}
*/


/*
void prvAvgGyro(){
  int sum[3] = {0,0,0};
  int avg[3] = {0,0,0};
  unsigned int sumCount = 0;
  while(1){
    sensors.readGyroscope(gyro[0], gyro[1], gyro[2]);
    sumCount++;
    for(int j=0; j<3; j++){
      sum[j] += gyro[j];
      avg[j] = sum[j]/sumCount;
    }
    Serial.print("Avg: ");
    Serial.print(avg[0]);
    Serial.print(", ");
    Serial.print(avg[1]);
    Serial.print(", ");
    Serial.println(avg[2]);
    delay(DELAY_MS);
  }
}
*/

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
//#include "SAAM.hh"
#include "InternalSensors.hh"
#include "BZZT.hh"
#include "SimulinkData.hh"
#include "SerialSpoofStepper.hh"
#include "QuickSilver.hh"
#include "Filter.hh"

#define LAUNCH_THRESHOLD_A_M_S2 10
#define LAUNCH_THRESHOLD_H_M 20
#define MIN_LOOPS_IN_STATE 3

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
float newAcc=0;
float vel=0;
float oldAcc=0;
float h=0;
float oldH=0;
float ang=0;
float h_groundLevel=0;

float a_raw[3] = {0,0,0};
float g_raw[3] = {0.0f, 0.0f, 0.0f};
float m[3] = {0,0,0};
float a[3] = {0,0,0};
float dt = 1000.0 / SENSOR_AND_CONTROL_DELAY_MS; //cannot be 0 or else problems :) 0.1 is the current nominal value

pt1Filter acc_filter[3];
pt1Filter simIn_filter;
QuickSilver attitude_estimate;

void setup(){
  Serial.begin(115200);
  while(!Serial);
  Serial.print("current dt: ");
  Serial.println(dt);
  longBzzt(1); //1 long means we are in setup
  delay(1000);

  simIn.startupTasks("TEST5.CSV");

    // initialize the acc_filters
  for (int axis = 0; axis < 3; axis++) {
    acc_filter[axis].init(5.0, dt); // TODO dt fed in here should be the rate at which we read new acc data
  }
  simIn_filter.init(5.0, dt);

  attitude_estimate.initialize(0.05); // TODO tune beta to a reasonable value
  //bta is how much closer we approach the accelerometer per loop. Longer time on pad? prolly lower beta. This is slower motion and therefore calibrates while on the pad and resists the initial acceleration from the rocket. 0.05 is 5% and it will take at 10 Hz about 10 or 15 seconds to converge the gyro to the accelerometer. beta brings gyro to acc. lower value might have bad things with noise, higher value is too sensitive
  
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

//this implementation of debounces prevents run-through but does not prevent
//a random noisy signal from triggering the next phase
void determineState(){
  int i;
  for(i=0; (newAcc < LAUNCH_THRESHOLD_A_M_S2 || h < LAUNCH_THRESHOLD_H_M ) || (i<MIN_LOOPS_IN_STATE); i++){
    //Serial.println("PRE");
    rocketState = ROCKET_PRE;
    delay(STATE_CHECKING_DELAY_MS);
  }
  for(i=0; (newAcc > 0) || (i<MIN_LOOPS_IN_STATE); i++){
    //Serial.println("LAUNCH");
    rocketState = ROCKET_LAUNCH;
    delay(STATE_CHECKING_DELAY_MS);
  }
  for(i=0; (vel>0) || (i<MIN_LOOPS_IN_STATE); i++){
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
  sensors.readGyroscope(g_raw[0], g_raw[1], g_raw[2]);
  float tempH=0;
  sensors.readAltitude(tempH);
  //convert H to AGL
  //Serial.print("H from Sensor:");
  //Serial.println(tempH - h_groundLevel);
  //convert A to m/s2
  //Serial.print("A from Sensors:");
  //Serial.println(a_raw[2] * G_TO_M_S2); //TODO: if sensor fusion works, then change this!

    //filter the acc data
  for (int axis = 0; axis < 3; axis++) {
    //a_raw[axis] = acc_filter[axis].apply(a_raw[axis]);
    a[axis] = acc_filter[axis].apply(a_raw[axis]);
  }

  // TODO read the gyro values
  float t = ((float)(simTimer.elapsed_time().count()))/1000000.0f;
  h = simIn.getInterpolatedAltitude(t);
  newAcc = simIn.getInterpolatedAcceleration(t);
  //Serial.print("h=");
  //Serial.println(h);
  //Serial.print("newAcc=");
  //Serial.println(newAcc);

  //filter the simulink data
  Serial.print("Raw newAcc: ");
  Serial.print(newAcc);
  newAcc = simIn_filter.apply(newAcc);
  Serial.print(" | Filtered newAcc: ");
  Serial.println(newAcc);
}
void prvIntegrateAccel(){
  //Serial.println("Entering prvIntegrateAccel");
  tNow = ((float)(tim.elapsed_time().count()))/1000000.0f;
  dt = (tNow - tOld);

  float fusion_gain = 0.5; // how much we trust accelerometer data

  float acc_integration = newAcc * dt; // will drift, but accurate over short times
  float barometer_derivative = (h - oldH) / dt;
  vel = fusion_gain * (vel + acc_integration) + (1.0 - fusion_gain) * barometer_derivative;


  //Serial.print("vel=");
  //Serial.println(vel);
}
void prvSensorFusion(){
  //attitude_estimate.update_estimate(a_raw, g_raw, dt); // TODO ensure that a_raw is in G's and that g_raw is in rad/s, and that dt is in seconds
  //float a_m_s[3] = {a_raw[0] * G_TO_M_S2, a_raw[1] * G_TO_M_S2, a_raw[2] * G_TO_M_S2};
  //newAcc = attitude_estimate.vertical_acceleration_from_acc(a_m_s); // TODO a_m_s here should be in m/^2, ensure that it is
  attitude_estimate.update_estimate(a, g_raw, dt); // TODO ensure that a_raw is in G's and that g_raw is in rad/s, and that dt is in seconds
  float a_m_s[3] = {a[0] * G_TO_M_S2, a[1] * G_TO_M_S2, a[2] * G_TO_M_S2};
  //newAcc = attitude_estimate.vertical_acceleration_from_acc(a_m_s); // TODO a_m_s here should be in m/^2, ensure that it is

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
  // todo fuse barometer with this
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
#include "InternalSensors.hh"
#include "BZZT.hh"
#include "QuickSilver.hh"
#include "Filter.hh"

#define GYRO_BIAS_X 0.138
#define GYRO_BIAS_Y 0.122
#define GYRO_BIAS_Z -0.226
// #define ACC_BIAS_X -0.0012
// #define ACC_BIAS_Y 0.0015
// #define ACC_BIAS_Z 0.0040

#define LAUNCH_THRESHOLD_A_M_S2 10
#define LAUNCH_THRESHOLD_H_M 20
#define MIN_LOOPS_IN_STATE 3
#define MIN_POINTS_TO_LEAVE 3

#define G_TO_M_S2 9.8f

void prvReadSensors();
void prvIntegrateAccel();
void prvDoControl();
void prvUpdateVars();

//we need a new timer because they last 30 minutes before they overflow.
//If we sit on the pad for longer than that then we don't know if we are just barely
//in the new 30 minutes or if we are going to overflow mid-flight. To avoid this,
//we just reset the timer until it is finally time to run. That way, we are garunteed
//to not overflow during launch
mbed::Timer tim;

RealStepper stepper;
SDLogger sd;
InternalSensors sensors;

float tNow=0;
float tOld=0;
float newAcc=0;
float vel=0;
float oldAcc=0;
float h=0;
float oldH=0;
float ang=0;
float h_groundLevel=0;

int h_resetCounter=0;

float a_raw[3] = {0,0,0};
float g_raw[3] = {0.0f, 0.0f, 0.0f};
float dt = 0.01;

#define BACK_ACC_LENGTH 10
float backAcc[BACK_ACC_LENGTH] = {0};
float backDt[BACK_ACC_LENGTH] = {0};
bool backCalcDone = false;

//pt1Filter acc_filter[3];
QuickSilver attitude_estimate;
pt1Filter gyroFilters[3];

void setup(){
  //Serial.begin(115200);
  //while(!Serial);
  longBzzt(1); //1 long means we are in setup
  delay(1000);

  // initialize the acc_filters
  for (int axis = 0; axis < 3; axis++) {
    gyroFilters[axis].init(5.0, 0.1); // TODO dt fed in here should be the rate at which we read new acc data
  }

  attitude_estimate.initialize(0.05); // TODO tune beta to a reasonable value

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
  int stayCounter = 0;
  bool proceed = false;
  int leaveCounter = 0;
  //for(i=0; (newAcc < LAUNCH_THRESHOLD_A_M_S2 || h < LAUNCH_THRESHOLD_H_M ) || (i<MIN_LOOPS_IN_STATE); i++){
  while(proceed == false){
    stayCounter++;
    //Serial.println("PRE");
    rocketState = ROCKET_PRE;
    delay(STATE_CHECKING_DELAY_MS);
    bool stay = (newAcc < LAUNCH_THRESHOLD_A_M_S2 || h < LAUNCH_THRESHOLD_H_M ) || (stayCounter<MIN_LOOPS_IN_STATE);
    if (!stay) leaveCounter++;
    if((!stay) && (leaveCounter > MIN_POINTS_TO_LEAVE)) proceed = true;
    if(stay) leaveCounter = 0;
    //if (stayCounter > 100) proceed = true;
  }
  proceed = false;
  stayCounter = 0;
  leaveCounter = 0;
  //for(i=0; (newAcc > 0) || (i<MIN_LOOPS_IN_STATE); i++){
  while(proceed == false){
    stayCounter++;
    //Serial.println("LAUNCH");
    rocketState = ROCKET_LAUNCH;
    delay(STATE_CHECKING_DELAY_MS);
    bool stay = (newAcc > 0) || (stayCounter<MIN_LOOPS_IN_STATE);
    if (!stay) leaveCounter++;
    if((!stay) && (leaveCounter > MIN_POINTS_TO_LEAVE)) proceed = true;
    if(stay) leaveCounter = 0;
  }
  proceed = false;
  stayCounter = 0;
  leaveCounter = 0;
  //for(i=0; (vel>0) || (i<MIN_LOOPS_IN_STATE); i++){
  while(proceed == false){
    stayCounter++;
    //Serial.println("FREEFALL");
    rocketState = ROCKET_FREEFALL;
    delay(STATE_CHECKING_DELAY_MS);
    bool stay = (vel > 0) || (stayCounter<MIN_LOOPS_IN_STATE);
    if (!stay) leaveCounter++;
    if((!stay) && (leaveCounter > MIN_POINTS_TO_LEAVE)) proceed = true;
    if(stay) leaveCounter = 0;
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

  if(++h_resetCounter > 10)
  {
    //h_groundLevel += h; //when in pre-flight, update the ground level every 10th pass
    h_resetCounter=0;
  }

  prvUpdateVars();
}
void sensorAndControl_LAUNCH(){
  //get data
  prvReadSensors();

  //integrate acc to get vel
  prvIntegrateAccel();

  //update variables
  prvUpdateVars();
}
void sensorAndControl_FULL(){
  //get data
  prvReadSensors();

  //fuse sensors to get vel
  prvIntegrateAccel();

  //set control value
  prvDoControl();

  //update variables
  prvUpdateVars();
}


void logging_RUN(){
  //Serial.println("Entering logging_RUN");
  sd.writeLog(a_raw[0], a_raw[1], a_raw[2], g_raw[0], g_raw[1], g_raw[2], newAcc, h_groundLevel, vel, ang, h, tNow, rocketState);
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
  tNow = ((float)(tim.elapsed_time().count()))/1000000.0f;
  // TODO get dt based on the time between last sample reads, not time since running this, it'll be more accurate this way
  dt = (tNow - tOld);
  //Serial.println("Entering prvReadSensors");
  sensors.readAcceleration(a_raw[0], a_raw[1], a_raw[2]);
  // a_raw[0] -= ACC_BIAS_X;
  // a_raw[0] -= ACC_BIAS_Y;
  // a_raw[0] -= ACC_BIAS_Z;
  sensors.readGyroscope(g_raw[0], g_raw[1], g_raw[2]);
  g_raw[0] -= GYRO_BIAS_X;
  g_raw[1] -= GYRO_BIAS_Y;
  g_raw[2] -= GYRO_BIAS_Z;
  g_raw[0] = gyroFilters[0].apply(g_raw[0]);
  g_raw[1] = gyroFilters[1].apply(g_raw[1]);
  g_raw[2] = gyroFilters[2].apply(g_raw[2]);

  //filter the acc data
  for (int axis = 0; axis < 3; axis++) {
    //a_raw[axis] = acc_filter[axis].apply(a_raw[axis]);
  }
  // TODO read the gyro values
  // TODO read the gyro values
  //sensors.readMagneticField(m[0], m[1], m[3]); // magnometer not needed
  float tempH=0;
  sensors.readAltitude(tempH);
  //convert H to AGL
  h = tempH -h_groundLevel;
  //Serial.print("H:");
  //Serial.println(h);
  //convert A to m/s2
  /*attitude_estimate.update_estimate(a_raw, g_raw, dt)
  float a_ms2[3];
  for(int i=0; i<3; i++){
    a_ms2[i] = a_raw[i] * G_TO_M_S2;
  }
  newAcc = attitude_estimate.vertical_acceleration_from_acc(a_ms2);*/
  //Serial.print("A:");
  //Serial.println(a_raw[2]);

  attitude_estimate.update_estimate(a_raw, g_raw, dt); 
  //float a_m_s[3] = {a_raw[0] * G_TO_M_S2, a_raw[1] * G_TO_M_S2, a_raw[2] * G_TO_M_S2};
  newAcc = attitude_estimate.vertical_acceleration_from_acc(a_raw) * G_TO_M_S2; //keep in Gs until the last second

}

void prvIntegrateAccel(){
  if(!backCalcDone){
    for(int i=BACK_ACC_LENGTH-1; i>=0; i--){
      vel += backAcc[i] * backDt[i];
    }
    backCalcDone = true;
  }
  //Serial.println("Entering prvIntegrateAccel");

  //make gain a function of vel?
  float Ma = constrain(vel/343.0f, 0.0, 1.0);
  float max_baro_trust = 0.8;
  //float fusion_gain = Ma * max_baro_trust + (1.0 - Ma) * (1.0 - max_baro_trust);
  float fusion_gain = 0.8;

  float acc_integration = newAcc * dt; // will drift, but accurate over short times
  float barometer_derivative = (h - oldH) / dt;
  vel = fusion_gain * (vel + acc_integration) + (1.0 - fusion_gain) * barometer_derivative;

}
void prvDoControl(){
  ang = getControl(getDesired(tNow), predictAltitude(h,vel), tNow-tOld);
  stepper.setStepsTarget(microStepsFromFlapAngle(ang));
}
void prvUpdateVars(){
  //Serial.println("Entering updateVars");
  oldAcc = newAcc; 
  tOld = tNow;
  oldH = h;

  backAcc[0] = oldAcc;
  backDt[0] = dt;
  for(int i=1; i<BACK_ACC_LENGTH; i++){
    backAcc[i] = backAcc[i-1];
    backDt[i] = backDt[i-1];
  }
}


#endif //FLIGHT



/*****************************************************************************
 * SENSORTEST
*****************************************************************************/
#ifdef SENSORTEST

//#include "RocketRTOS.hh"
//#include "SerialSpoofStepper.hh"
#include "Control.hh"
#include "ExternalSensors.hh"
//#include "SDSpoofer.hh"
#include "SDLogger.hh"

//SDSpoofer dummySD;
SDLogger sd; //seeing if this is causing a runtime error//jonse
//SerialSpoofStepper stepper;
ExternalSensors intSensors; //marbe: yeah it makes shit-all for sense but I'm testing here


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

  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial Setup Complete");
  intSensors.startupTasks();
  Serial.println("Sensor Startup Complete");
  //dummySD.openFile();
  sd.openFile();

 // startRocketRTOS();
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

  unsigned long t0 = 0;
  unsigned long tnow = 0;
String bufferedline = "";
int n_max = 1000;
//how fast we goin herrre....
void loop(){
  unsigned long del_t = 0;
  for (int i = 0; i < n_max; i++)  {
  t0 = micros();
  intSensors.readAcceleration(accelerometer_data_x, accelerometer_data_y, accelerometer_data_z);
  intSensors.readPressure(pressure_data);
  intSensors.readTemperature(temperature_data);
  intSensors.readMagneticField(magfield_data_x, magfield_data_y, magfield_data_z);
  intSensors.readGyroscope(gyro_data_x, gyro_data_y, gyro_data_z);
  intSensors.readAltitude(altitude_data);
  tnow=micros();
  bufferedline.append(String(t0)+',');
  bufferedline.append(String(accelerometer_data_x)+','+ String(accelerometer_data_y)+','+ String(accelerometer_data_z)+',');
  bufferedline.append(String(gyro_data_x)+','+ String(gyro_data_y)+','+ String(gyro_data_z)+',');
  bufferedline.append(String(magfield_data_x)+','+ String(magfield_data_y)+','+ String(magfield_data_z)+',');
  bufferedline.append(String(pressure_data)+','+ String(temperature_data)+','+ String(altitude_data)+'\n');
  del_t+=tnow-t0;
  }
  del_t = del_t/(n_max);
  //Serial.println("Average loop time: "+String(del_t)+" us");
  t0 = micros();
  sd.writeLine(bufferedline);
  tnow = micros();
  Serial.println("Write time for "+String(n_max)+" lines of data: "+String(tnow-t0)+" us");
  bufferedline="";
  sd.closeFile();

}



void displayOrientationAltitude_SDSpoof(){ //patmer Display results over serial with SDSpoofer
  Serial.println("Displaying data to SDSpoofer");

  sd.writeLog(accel, vel, h, ang);
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

/*****************************************************************************
 * TEENSY_4_1_TESTING
*****************************************************************************/
#ifdef TEENSY_4_1_TESTING

#include "RocketRTOS.hh"
#include "InterruptingStepper.hh"
// #include "SDLogger.hh"
#include "Control.hh"
#include "QuickSilver.hh"
#include "Filter.hh"
#include "StateMachine.hh"
#include "ExternalSensors.hh"
//#include "SimulinkData.hh"
#include "SDSpoofer.hh"
#include <IntervalTimer.h>
#include <climits>
// #include <InterruptingBuzzer.hh>
// #define BUZZ_PIN 39
#define BUZZ_PIN 38 //To disable the buzzer so I can code in public :)
#define BUZZ_TIME 250000 //0.25 sec
#define PAUSE_SHORT 500000 //0.5 sec
// #define PAUSE_LONG 5000000 //5.0 sec
#define PAUSE_LONG 2000000 //2.0 sec
// IntervalTimer buzzerTicker;
// bool doBuzz = false;
// int buzzerMicros = 500000;
// #define BZZT_PIN 39
// void prvBuzzer();

// #define GYRO_BIAS_X 0.998988f
// #define GYRO_BIAS_Y 0.574029f
// #define GYRO_BIAS_Z -12.5213f
// #define GYRO_BIAS_X 1.91 //these are now taken care of by the ExternalSensors class
// #define GYRO_BIAS_Y 5.89
// #define GYRO_BIAS_Z 5.43
// #define ACC_BIAS_X -0.0012
// #define ACC_BIAS_Y 0.0015
// #define ACC_BIAS_Z 0.0040

#define LAUNCH_THRESHOLD_A_M_S2 10
#define LAUNCH_THRESHOLD_H_M 20
#define MIN_LOOPS_IN_STATE 3
#define MIN_POINTS_TO_LEAVE 3

#define G_TO_M_S2 9.8f

InterruptingStepper stepper;
// SDLogger sd;
SDSpoofer sd;
// SimulinkFile simIn;
StateMachine state;
ExternalSensors sensors;

void prvReadSensors();
void prvIntegrateAccel();
void prvDoControl();
void prvUpdateVars();

unsigned long microsNow=0;
unsigned long microsOld = 0;
unsigned long deltaMicros = 1;
unsigned long testStartMicros = 0;
unsigned long burnoutMicros = 0;
unsigned long buzzMicros = 0;
unsigned long altimeterMicros = 0;
float burnoutTime=0; //time since burnout
float simTime = 0;
float newAcc=0;
float vel=0;
float h_raw=0;
float oldH=0;
float ang=0;
float h_groundLevel=0;
float intA=0;
float diffH=0;
float h_filtered=0;

int h_resetCounter=0;

float a_raw[3] = {0.0f, 0.0f, 0.0f};
float a_filtered[3] = {0.0f, 0.0f, 0.0f};
float g_raw[3] = {0.0f, 0.0f, 0.0f};
float g_filtered[3] = {0.0f, 0.0f, 0.0f};
float dt = 0.001;
float dt_h = 0.023; //baro runs at 50 Hz (x8 oversampling -> 22.5 ms (19.5 ms typ) read time + 0.5 ms standby setting) //from datasheet :)

#define BACK_ACC_LENGTH 10
float backAcc[BACK_ACC_LENGTH] = {0};
float backDt[BACK_ACC_LENGTH] = {0};
bool backCalcDone = false;

float desiredH = 0;
float predictedH = 0;

//pt1Filter acc_filter[3];
QuickSilver attitude_estimate;
pt1Filter gyroFilters[3];
pt1Filter accFilters[3];
pt1Filter hFilter;

void setup(){
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial Connected");
  // buzzerTicker.priority(130);
  // buzzerTicker.begin(prvBuzzer, buzzerMicros);

  // initialize the filteres
  for (int axis = 0; axis < 3; axis++) {
    gyroFilters[axis].init(1.0, 0.001); // TODO dt fed in here should be the rate at which we read new acc data
    accFilters[axis].init(5.0, 0.001);
  }
  hFilter.init(5.0, 0.001);

  attitude_estimate.initialize(0.05); // TODO tune beta to a reasonable value

  // Serial.println("Reading from simulation file");
  // simIn.startupTasks("TEST12.CSV");
  // simIn.printData();

  sensors.startupTasks();
  sensors.readAltitude(h_groundLevel);

  // initializeBuzzer();
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, 1);
  delay(1000);
  digitalWrite(BUZZ_PIN, 0);

  // sd.openFile("Acc, Vel, h_raw, h_filtered, h_ground, Ang, simT, burnoutT, State, DesiredH, PredictedH, intA, diffH, dt, 1/dt, a_raw[0], a_raw[1], a_raw[2], a_filtered[0], a_filtered[1], a_filtered[2], g_raw[0], g_raw[1], g_raw[2], g_filtered[0], g_filtered[1], g_filtered[2], grav[0], grav[1], grav[2]");
  sd.openFile("AX, AY, AZ, GX, GY, GZ");

  delay(500);

  stepper.start();
  testStartMicros = micros();
  startRocketRTOS();
}



void determineState(){
  state.updateState(h_filtered, vel, newAcc);
}


void sensorAndControl_PRE(){
  prvReadSensors(); 

  // if(++h_resetCounter > 10)
  // {
  //   h_groundLevel += h; //when in pre-flight, update the ground level every 10th pass
  //   h_resetCounter=0;
  // }

  prvUpdateVars();
}
void sensorAndControl_LAUNCH(){
  burnoutMicros = micros(); //what to do if overflow during this?
  prvReadSensors();
  prvIntegrateAccel();
  prvUpdateVars();
}
void sensorAndControl_FULL(){
  prvReadSensors();
  prvIntegrateAccel();
  prvDoControl();
  prvUpdateVars();
}

void logging_RUN(){
  //Serial.println("log run");
  //sd.writeLog(newAcc, vel, h, ang, simTime, burnoutTime, rocketState);
  // sd.writeLog(String("A: ") + String(newAcc) + String(", V:") + String(vel) + String(", H: ") + String(h) + String(", Ang:") + String(ang) + String(", simTime:") + String(simTime) + String(", burnoutTime") + String(burnoutTime) + String(", State:") + String(rocketState) + String(", H_d:") + String(desiredH) + String(", H_p:") + String(predictedH));
  // String log = String(newAcc) + String(", ") + String(vel) + String(", ") + String(h_raw) + String(", ") + String(h_filtered) + String(", ") + String(h_groundLevel) + String(", ") 
  //           + String(ang) + String(", ") + String(simTime) + String(", ") + String(burnoutTime) + String(", ") + String(rocketState) + String(", ") 
  //           + String(desiredH) + String(", ") + String(predictedH) + String(", ") + String(intA) + String(", ") + String(diffH) + String(", ") 
  //           + String(dt) + String(", ") + String((1.0f/dt)) + String(", ") 
  //           + String(a_raw[0]) + String(", ") + String(a_raw[1]) + String(", ") + String(a_raw[2]) + String(", ") 
  //           + String(a_filtered[0]) + String(", ") + String(a_filtered[1]) + String(", ") + String(a_filtered[2]) + String(", ") 
  //           + String(g_raw[0]) + String(", ") + String(g_raw[1]) + String(", ") + String(g_raw[2]) + String(", ") 
  //           + String(g_filtered[0]) + String(", ") + String(g_filtered[1]) + String(", ") + String(g_filtered[2]) + String(", ")
  //           + String(attitude_estimate.getGravityVector()[0]) + String(", ") + String(attitude_estimate.getGravityVector()[1]) + String(", ") + String(attitude_estimate.getGravityVector()[2]);
  // static float sums[3] = {0.0,0.0,0.0};
  // static int count = 0;

  // for(int axis = 0; axis < 3; axis++){
  //   sums[axis] += g_raw[axis];
  // }
  // count++;
  
  // String log2 = String("GyroX:") + String(g_raw[0]) + String(", GyroY:") + String(g_raw[1]) + String(", GyroZ:") + String(g_raw[2]);
  // String log = String("GyroX:") + String(sums[0]/((float)count)) + String(", GyroY:") + String(sums[1]/((float)count)) + String(", GyroZ:") + String(sums[2]/((float)count));
  // String log = String("GravX:") + String(attitude_estimate.getGravityVector()[0]) + String(", GravY:") + String(attitude_estimate.getGravityVector()[1]) + String(", GravZ:") + String(attitude_estimate.getGravityVector()[2]);
  // String log1 = String("AccX:") + String(a_raw[0]) + String(", AccY:") + String(a_raw[1]) + String(", AccZ:") + String(a_raw[2]);
  // String log = log1 + String("; ") + log2;
  String log = String("VertAcc: ") + String(newAcc);
  // String log = String(a_raw[0]) + String(", ") + String(a_raw[1]) + String(", ") + String(a_raw[2]) + String(", ") + String(g_raw[0]) + String(", ") + String(g_raw[1]) + String(", ") + String(g_raw[2]);
  sd.writeLine(log);
}
void logging_CLOSE(){
  //Serial.println("log close");
  sd.closeFile();
}
void logging_IDLE(){
  //Serial.println("log idle");
}

void stepper_RUN(){
  //Serial.println("step run");
  // if(printStepHigh){
  //   Serial.println("StepUp");
  //   printStepHigh = false;
  // }
  // if(printStepLow){
  //   Serial.println("StepDown");
  //   printStepLow = false;
  // }

  // Serial.print("Current Step: ");
  // Serial.println(currentStepGlobal);
  // Serial.print("Step Goal: ");
  // Serial.println(stepsTargetGlobal);
}
void stepper_CLOSE(){
  stepper.setStepsTarget(0);
}
void stepper_IDLE(){
  //Serial.println("step idle");
}

void buzz_PRE(){
  // doBuzz = true;
  // buzzerMicros = 1000000;
  //Serial.println("buzz pre");
  //Serial.println("BZZZZZT");
  // enablePreBuzz();

  unsigned long localDiff = micros() - buzzMicros;
  if(localDiff > (BUZZ_TIME+PAUSE_LONG)){
    digitalWrite(BUZZ_PIN, 1);
    buzzMicros = micros();
  } else if(localDiff > BUZZ_TIME){
    digitalWrite(BUZZ_PIN, 0);
  }
}
void buzz_POST(){
  // doBuzz = true;
  // buzzerMicros = 250000;
  //Serial.println("buzz post");
  //Serial.println("BZZZT BZZT");
  // enablePostBuzz();
  unsigned long localDiff = micros() - buzzMicros;
  if(localDiff > (BUZZ_TIME+PAUSE_SHORT+BUZZ_TIME+PAUSE_LONG)){
    digitalWrite(BUZZ_PIN, 1);
    buzzMicros = micros();
  } else if(localDiff > (BUZZ_TIME+PAUSE_SHORT+BUZZ_TIME)){
    digitalWrite(BUZZ_PIN, 0);
  } else if(localDiff > (BUZZ_TIME+PAUSE_SHORT)){
    digitalWrite(BUZZ_PIN, 1);
  } else if(localDiff > BUZZ_TIME){
    digitalWrite(BUZZ_PIN, 0);
  }
}
void buzz_IDLE(){
  // disableBuzz();
  //Serial.println("buzz idle");
  // doBauzz = false;
  digitalWrite(BUZZ_PIN, 0);
}




void prvReadSensors(){


  //Serial.println("Entering prvReadSensors");
  sensors.readAcceleration(a_raw[0], a_raw[1], a_raw[2]);
  sensors.readGyroscope(g_raw[0], g_raw[1], g_raw[2]);
  // g_raw[0] -= GYRO_BIAS_X;
  // g_raw[1] -= GYRO_BIAS_Y;
  // g_raw[2] -= GYRO_BIAS_Z;
  // a_raw[0] -= ACC_BIAS_X;
  // a_raw[1] -= ACC_BIAS_Y;
  // a_raw[2] -= ACC_BIAS_Z;

  // //float tempH=0;
  // sensors.readAltitude(h_raw);
  // //convert H to AGL
  // h_raw -= h_groundLevel;
  //Serial.print("H from Sensor:");
  //Serial.println(tempH - h_groundLevel);
  //convert A to m/s2
  //Serial.print("A from Sensors:");
  //Serial.println(a_raw[2] * G_TO_M_S2); //TODO: if sensor fusion works, then change this!

  //Apply Filters
  for (int axis = 0; axis < 3; axis++) {
    a_filtered[axis] = accFilters[axis].apply(a_raw[axis]);
    g_filtered[axis] = gyroFilters[axis].apply(g_raw[axis]);
  }

  //read h and calculate diffH when a new value of h is ready
  if( (micros() - altimeterMicros) > 23000){ //19.5 ms is the typical pressure read case. Worst case is 22.5 ms. Standby is 0.5 ms
    sensors.readAltitude(h_raw);
    h_raw -= h_groundLevel;

    oldH = h_filtered;
    h_filtered = hFilter.apply(h_raw);
    dt_h = micros() - altimeterMicros;
    altimeterMicros = micros();
    diffH = (h_filtered - oldH) / dt;
  }

  simTime = ((float)( micros() - testStartMicros)) / 1000000.0f;
  // h = simIn.getInterpolatedAltitude(simTime);
  // newAcc = simIn.getInterpolatedAcceleration(simTime);


  //Attitude Determination
  //Using Raw
  // attitude_estimate.update_estimate(a_raw, g_raw, dt); // TODO ensure that a_raw is in G's and that g_raw is in rad/s, and that dt is in seconds
  // float a_m_s[3] = {a_raw[0] * G_TO_M_S2, a_raw[1] * G_TO_M_S2, a_raw[2] * G_TO_M_S2};
  // newAcc = attitude_estimate.vertical_acceleration_from_acc(a_m_s); // TODO a_m_s here should be in m/^2, ensure that it is
  //Using FIltered
  attitude_estimate.update_estimate(a_filtered, g_filtered, dt); // TODO ensure that a_raw is in G's and that g_raw is in rad/s, and that dt is in seconds
  newAcc = attitude_estimate.vertical_acceleration_from_acc(a_filtered);
  newAcc *= G_TO_M_S2;



}
void prvIntegrateAccel(){
  if(!backCalcDone){
    for(int i=BACK_ACC_LENGTH-1; i>=0; i--){
      // vel += backAcc[i] * backDt[i];
      intA += backAcc[i] * backDt[i];
    }
    backCalcDone = true;
  }
  burnoutTime = ((float)( micros() - burnoutMicros )) / 1000000.0f;
  //Serial.println("Entering prvIntegrateAccel");
  //tNow = ((float)(tim.elapsed_time().count()))/1000000.0f;

  microsNow = micros();
  //calculate dt but catch any overflow error
  if(microsNow > microsOld){
    deltaMicros = (microsNow - microsOld);
  } else {
    deltaMicros = (ULONG_MAX - microsOld) - microsNow;
  }

  dt = ((float)deltaMicros) / 1000000.0f;

  float fusion_gain = 0.5; // how much we trust accelerometer data

  intA = newAcc * dt; // will drift, but accurate over short times
  // diffH = (h_filtered - oldH) / dt; //moved to prvReadSensors
  vel = fusion_gain * (vel + intA) + (1.0 - fusion_gain) * diffH;


  //Serial.print("vel=");
  //Serial.println(vel);
}
void prvDoControl(){
  desiredH = getDesired(burnoutTime);
  predictedH = predictAltitude(h_filtered,vel);
  // ang = getControl(getDesired(burnoutTime), predictAltitude(h,vel), dt);
  ang = getControl(desiredH, predictedH, dt);
  stepper.setStepsTarget(microStepsFromFlapAngle(ang));
}
void prvUpdateVars(){
  //Serial.println("Entering updateVars");
  // oldH = h_filtered; //this one was moved to when the filter is applied
  // oldH_raw = h_raw;
  microsOld = microsNow;


  backAcc[0] = newAcc;
  backDt[0] = dt;
  for(int i=1; i<BACK_ACC_LENGTH; i++){
    backAcc[i] = backAcc[i-1];
    backDt[i] = backDt[i-1];
  }
}


// void prvBuzzer(){
//   if(doBuzz){
//     digitalToggleFast(BZZT_PIN);
//   }
// }



#endif //TEENSY_4_1_TESTING

