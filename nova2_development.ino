#include <Arduino.h>
/******************************************************************************
 * nova2_development.ino
 * 
 * Contains the runnable code for development, testing, and flight
 * 
 * Uncomment one of the following defines to choose which test to run
******************************************************************************/
#define DEVELOPMENT
//#define STATE_TEST
//#define CONTROL_TEST
//#define FLIGHT
//#define SENSORTEST

/*****************************************************************************
 * DEVELOPMENT
*****************************************************************************/
#ifdef DEVELOPMENT

#include "QuickSilver.hh"
#include "InternalSensors.hh"
#include "Filter.hh"
#include "mbed.h"
#include "rtos.h"

#define BETA 0.1f
#define DELAY_MS 100

//Gyro calibration bias. These change with temperature
#define GYRO_BIAS_X 0.00
#define GYRO_BIAS_Y 0.00
#define GYRO_BIAS_Z 0.00

QuickSilver attitude;
InternalSensors sensors;
pt1Filter filter;
//mbed::Ticker tick;
rtos::Thread gyroThread(osPriorityAboveNormal, 1024);

void prvFilterGyro();
#define GRYO_FILTER_PERIOD_MS 10 //100 Hz

float tOld = 0.0f;
float tNow = 0.0f;

float acc[3] = {0.0f, 0.0f, 0.0f};
float gyro[3] = {0.0f, 0.0f, 0.0f};

void setup(){
  Serial.begin(115200);
  while(!Serial);

  filter.init(5.0f, 0.1f);
  sensors.startupTasks();
  attitude.initialize(BETA);
  tNow = ((float)(micros()))/1000000.0f;
  //tick.attach(&prvFilterGyro, GRYO_FILTER_PERIOD);
  gyroThread.start(prvFilterGyro);
}

void loop(){
  delay(DELAY_MS);
  sensors.readAcceleration(acc[0], acc[1], acc[2]);
  /*sensors.readGyroscope(gyro[0], gyro[1], gyro[2]);
  gyro[0] -= GYRO_BIAS_X;
  gyro[1] -= GYRO_BIAS_Y;
  gyro[2] -= GYRO_BIAS_Z;
  gyro[0] = filter.apply(gyro[0]);
  gyro[1] = filter.apply(gyro[1]);
  gyro[2] = filter.apply(gyro[2]);*/
  Serial.print("Acc: ");
  Serial.print(acc[0]);
  Serial.print(", ");
  Serial.print(acc[1]);
  Serial.print(", ");
  Serial.print(acc[2]);
  Serial.println();
  Serial.print("Gyro: ");
  Serial.print(gyro[0]);
  Serial.print(", ");
  Serial.print(gyro[1]);
  Serial.print(", ");
  Serial.print(gyro[2]);
  Serial.println();

  tOld = tNow;
  tNow = ((float)(micros()))/1000000.0f;
  float dt = tNow - tOld;
  attitude.update_estimate(acc,gyro,dt);
  float *grav = attitude.getGravityVector();
  Serial.print("Grav: ");
  Serial.print(grav[0]);
  Serial.print(", ");
  Serial.print(grav[1]);
  Serial.print(", ");
  Serial.print(grav[2]);
  Serial.println();

  float vert = attitude.vertical_acceleration_from_acc(acc);
  Serial.print("Vert: ");
  Serial.println(vert);

  Serial.print("tNow: ");
  Serial.println(tNow);
}


//causes hard fault???
//probably because reading the sensors is I2C and blocking and otherwise not ISR safe

void prvFilterGyro(){
  sensors.readGyroscope(gyro[0], gyro[1], gyro[2]);
  gyro[0] -= GYRO_BIAS_X;
  gyro[1] -= GYRO_BIAS_Y;
  gyro[2] -= GYRO_BIAS_Z;
  gyro[0] = filter.apply(gyro[0]);
  gyro[1] = filter.apply(gyro[1]);
  gyro[2] = filter.apply(gyro[2]);

  delay(GRYO_FILTER_PERIOD_MS);
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
//#include "SAAM.hh"
#include "InternalSensors.hh"
#include "BZZT.hh"
#include "Madgwick.hh"
#include "Filter.hh"

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

float a_raw[3] = {0,0,0};
float g_raw[3] = {0.0f, 0.0f, 0.0f};
float g_raw[3] = {0.0f, 0.0f, 0.0f};
float m[3] = {0,0,0};
float a[3] = {0,0,0};
float dt = 0.01;
float dt = 0.01;

pt1Filter acc_filter[3];
Madgwick attitude_estimate;
Madgwick attitude_estimate;

void setup(){
  //Serial.begin(115200);
  //while(!Serial);
  longBzzt(1); //1 long means we are in setup
  delay(1000);

  // initialize the acc_filters
  for (int axis = 0; axis < 3; axis++) {
    acc_filter[axis].init(5.0, 0.01); // TODO dt fed in here should be the rate at which we read new acc data
  }

  attitude_estimate.initialize(0.05); // TODO tune beta to a reasonable value

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
  while(newAcc < LAUNCH_THRESHOLD_A_M_S2 || h < LAUNCH_THRESHOLD_H_M ){
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

  //fuse sensors to get vel
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
  //filter the acc data
  for (int axis = 0; axis < 3; axis++) {
    a_raw[axis] = acc_filter[axis].apply(a_raw[axis]);
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
  newAcc = a_raw[2] * G_TO_M_S2;
  //Serial.print("A:");
  //Serial.println(a_raw[2]);

}
void prvIntegrateAccel(){
  //Serial.println("Entering prvIntegrateAccel");
  tNow = ((float)(tim.elapsed_time().count()))/1000000.0f;
  // TODO get dt based on the time between last sample reads, not time since running this, it'll be more accurate this way
  dt = (tNow - tOld);

  float fusion_gain = 0.2; // how much we trust accelerometer data

  float acc_integration = newAcc * dt; // will drift, but accurate over short times
  float barometer_derivative = (h - oldH) / dt;
  vel = fusion_gain * (vel + acc_integration) + (1.0 - fusion_gain) * barometer_derivative;

>>>>>>> 985398a (fuse barometer and acc to get velocity estimates)
  //Serial.print("vel=");
  //Serial.println(vel);
}
void prvSensorFusion(){
  attitude_estimate.update_estimate(a_raw, g_raw, dt); // TODO ensure that a_raw is in G's and that g_raw is in rad/s, and that dt is in seconds
  float a_m_s[3] = {a_raw[0] * G_TO_M_S2, a_raw[1] * G_TO_M_S2, a_raw[2] * G_TO_M_S2};
  newAcc = attitude_estimate.vertical_acceleration_from_acc(a_m_s); // TODO a_m_s here should be in m/^2, ensure that it is
  attitude_estimate.update_estimate(a_raw, g_raw, dt); // TODO ensure that a_raw is in G's and that g_raw is in rad/s, and that dt is in seconds
  float a_m_s[3] = {a_raw[0] * G_TO_M_S2, a_raw[1] * G_TO_M_S2, a_raw[2] * G_TO_M_S2};
  newAcc = attitude_estimate.vertical_acceleration_from_acc(a_m_s); // TODO a_m_s here should be in m/^2, ensure that it is
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



