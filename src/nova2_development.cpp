#include <Arduino.h>

// #define SIMULINK_TESTING
// #define WAIT_FOR_SERIAL


#include "RocketRTOS.hh"
#include "InterruptingStepper.hh"
#include "Control.hh"
#include "QuickSilver.hh"
#include "Filter.hh"
#include "StateMachine.hh"
#include <climits>

#ifdef SIMULINK_TESTING
  #include "SimulinkData.hh"
  #include "SDSpoofer.hh"
#else //NOT simulink testing
  #include "ExternalSensors.hh"
  #include "SDLogger.hh"
#endif //SIMULINK_TESTING

#define BUZZ_PIN 6
// #define BUZZ_PIN 5 //re-route buzzer to LED
#define BUZZ_TIME 125000 //0.125 sec
#define PAUSE_SHORT 500000 //0.5 sec
#define PAUSE_LONG 5000000 //5.0 sec
// #define PAUSE_LONG 2000000 //2.0 sec


#define LAUNCH_THRESHOLD_A_M_S2 10
#define LAUNCH_THRESHOLD_H_M 20
#define MIN_LOOPS_IN_STATE 3
#define MIN_POINTS_TO_LEAVE 3

#define G_TO_M_S2 9.8f

InterruptingStepper stepper;
StateMachine state;

#ifdef SIMULINK_TESTING
  SimulinkFile simIn;
  SDSpoofer sd;
#else
  ExternalSensors sensors;
  SDLogger sd;
#endif

inline void prvReadSensors();
inline void prvIntegrateAccel();
inline void prvDoControl();


static unsigned long microsNow=0;
static unsigned long microsOld = 0;
static unsigned long deltaMicros = 1;
static unsigned long testStartMicros = 0;
static unsigned long burnoutMicros = 0;
static unsigned long buzzMicros = 0;
static unsigned long altimeterMicros = 0;
static float burnoutTime=0; //time since burnout
float simTime = 0;
float newAcc=0;
float vel=0;
float h_raw=0;
float oldH=0;
float ang=0;
float h_groundLevel=-1.0f;
float intA=0;
float diffH=-1.0f;
float h_filtered=-1.0f;

float integralOfAccel = 0;

int h_resetCounter=0;

float a_raw[3] = {0.0f, 0.0f, 0.0f};
float a_filtered[3] = {0.0f, 0.0f, 0.0f};
float g_raw[3] = {0.0f, 0.0f, 0.0f};
float g_filtered[3] = {0.0f, 0.0f, 0.0f};
float dt = 0.01;
float dt_h = 0.023; //baro runs at 50 Hz (x8 oversampling -> 22.5 ms (19.5 ms typ) read time + 0.5 ms standby setting) //from datasheet :)

#define BACK_ACC_LENGTH 1000
float backAcc[BACK_ACC_LENGTH] = {0};
float backDt[BACK_ACC_LENGTH] = {0};
bool backCalcDone = false;
int backCalcIndex = 0;

float desiredH = 0;
float predictedH = 0;


//pt1Filter acc_filter[3];
QuickSilver attitude_estimate;
pt1Filter gyroFilters[3];
pt1Filter accFilters[3];
pt1Filter hFilter;

void setup(){
#ifdef WAIT_FOR_SERIAL
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial Connected");
#endif

  pinMode(BUZZ_PIN, OUTPUT);

//if the rocket will try to calibrate itself, beep 3 times quickly
#ifndef STATIC_OFFSETS
  digitalWrite(BUZZ_PIN, 1);
  delay(100);
  digitalWrite(BUZZ_PIN, 0);
  delay(100);
  digitalWrite(BUZZ_PIN, 1);
  delay(100);
  digitalWrite(BUZZ_PIN, 0);
  delay(100);
  digitalWrite(BUZZ_PIN, 1);
  delay(100);
  digitalWrite(BUZZ_PIN, 0);
#endif

  // initialize the filteres
  for (int axis = 0; axis < 3; axis++) {
    gyroFilters[axis].init(1.0, dt); // dt fed in here should be the rate at which we read new acc data
    accFilters[axis].init(5.0, dt);
  }
  hFilter.init(1.0, dt_h);

  attitude_estimate.initialize(0.005); // TODO tune beta to a reasonable value

#ifdef SIMULINK_TESTING
  Serial.println("Reading from simulation file");
  simIn.startupTasks("TEST15.CSV");
  //simIn.printData();
#endif

#ifndef SIMULINK_TESTING
  sensors.startupTasks();
  sensors.readAltitude(h_groundLevel);
#endif

#ifndef STATIC_OFFSETS
  while(1){
    digitalWrite(BUZZ_PIN, 1);
    delay(100);
    digitalWrite(BUZZ_PIN, 0);
    delay(100);
    digitalWrite(BUZZ_PIN, 1);
    delay(100);
    digitalWrite(BUZZ_PIN, 0);
    delay(100);
    digitalWrite(BUZZ_PIN, 1);
    delay(100);
    digitalWrite(BUZZ_PIN, 0);
    delay(500);
  }
#endif

  digitalWrite(BUZZ_PIN, 1);
  delay(1000);
  digitalWrite(BUZZ_PIN, 0);

  stepper.start();
  stepper.enable();
  stepper.setStepsTarget(microStepsFromFlapAngle(0.3));
  delay(1000);
  stepper.setStepsTarget(0);
  delay(500);
  stepper.disable();

  digitalWrite(BUZZ_PIN, 1);
  delay(1000);
  digitalWrite(BUZZ_PIN, 0);

#ifndef SIMULINK_TESTING
  sd.openFile("Acc, Vel, h_raw, h_filtered, h_ground, Ang, simT, burnoutT, State, DesiredH, PredictedH, intA, diffH, IntegralOfA, dt, 1/dt, dt_h, 1/dt_h, a_raw[0], a_raw[1], a_raw[2], a_filtered[0], a_filtered[1], a_filtered[2], g_raw[0], g_raw[1], g_raw[2], g_filtered[0], g_filtered[1], g_filtered[2], grav[0], grav[1], grav[2]");
#else
  sd.openFile("t, state, ang, desired, predicted, burnoutTime, burnoutMicros, testStartMicros, micros");
  //sd.openFile("t, state, stepsTarget, currentStep, direction");
#endif

  testStartMicros = micros();
  startRocketRTOS();

}
void determineState(){
  state.updateState(h_filtered, vel, newAcc);
}


void sensorAndControl_PRE(){
  prvReadSensors(); 
  if(++h_resetCounter > 100000) //every 100 seconds
  {
    h_groundLevel += h_filtered; //when in pre-flight, update the ground level every nth pass
    h_resetCounter=0;
  }
}
void sensorAndControl_LAUNCH(){
  burnoutMicros = micros(); //TODO: what to do if overflow during this? Overflow occurs just over an hour after power-on
  prvReadSensors();
  prvIntegrateAccel();
}
void sensorAndControl_FULL(){
  prvReadSensors();
  prvIntegrateAccel();
  prvDoControl();
}

void logging_RUN(){
#ifndef SIMULINK_TESTING
  String log = String(newAcc) + String(", ") + String(vel) + String(", ") + String(h_raw) + String(", ") + String(h_filtered) + String(", ") + String(h_groundLevel) + String(", ") 
            + String(ang) + String(", ") + String(simTime) + String(", ") + String(burnoutTime) + String(", ") + String(rocketState) + String(", ") 
            + String(desiredH) + String(", ") + String(predictedH) + String(", ") + String(intA) + String(", ") + String(diffH) + String(", ") + String(integralOfAccel) + String(", ")
            + String(dt) + String(", ") + String((1.0f/dt)) + String(", ")  + String(dt_h) + String(", ") + String(1.0f/dt_h) + String(", ")
            + String(a_raw[0]) + String(", ") + String(a_raw[1]) + String(", ") + String(a_raw[2]) + String(", ") 
            + String(a_filtered[0]) + String(", ") + String(a_filtered[1]) + String(", ") + String(a_filtered[2]) + String(", ") 
            + String(g_raw[0]) + String(", ") + String(g_raw[1]) + String(", ") + String(g_raw[2]) + String(", ") 
            + String(g_filtered[0]) + String(", ") + String(g_filtered[1]) + String(", ") + String(g_filtered[2]) + String(", ")
            + String(attitude_estimate.getGravityVector()[0]) + String(", ") + String(attitude_estimate.getGravityVector()[1]) + String(", ") + String(attitude_estimate.getGravityVector()[2]);
#else 
  String log = String(simTime) + String(", ") + String(rocketState) + String(", ") + String(ang) + String(", ") + String(desiredH) + String(", ") + String(predictedH) + String(", ") + String(burnoutTime) + String(", ") + String(burnoutMicros) + String(", ") + String(testStartMicros) + String(", ") + String(microsNow);
  //String log = String(simTime) + String(", ") + String(rocketState) + String(", ") + String(stepperVars.stepsTarget) + String(", ") + String(stepperVars.currentStep) + String(", ") + String(stepperVars.direction);
#endif

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
  // stepper.enable();
  digitalWriteFast(ENABLE_PIN, MOTOR_ENABLE);
}
void stepper_CLOSE(){
  //stepper.enable(); //we can assume that it went through run before getting to close, so no need to re-enable
  stepper.setStepsTarget(0);
}
void stepper_IDLE(){
  digitalWriteFast(ENABLE_PIN, MOTOR_DISABLE);
}

void buzz_PRE(){
  unsigned long localDiff = micros() - buzzMicros;
  if(localDiff > (BUZZ_TIME+PAUSE_LONG)){
    digitalWriteFast(BUZZ_PIN, 1);
    buzzMicros = micros();
  } else if(localDiff > BUZZ_TIME){
    digitalWriteFast(BUZZ_PIN, 0);
  }
}
void buzz_POST(){
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
  digitalWriteFast(BUZZ_PIN, 0);
}




inline void prvReadSensors(){
  microsNow = micros();
  simTime = ((float)( microsNow - testStartMicros)) / 1000000.0f;
  burnoutTime = ((float)( microsNow - burnoutMicros )) / 1000000.0f;

  //calculate dt but catch any overflow error
  if(microsNow > microsOld){
    deltaMicros = (microsNow - microsOld);
  } else {
    deltaMicros = (ULONG_MAX - microsOld) - microsNow;
  }

  dt = ((float)deltaMicros) / 1000000.0f;

  microsOld = microsNow;

#ifndef SIMULINK_TESTING
  sensors.readAcceleration(a_raw[0], a_raw[1], a_raw[2]);
  sensors.readGyroscope(g_raw[0], g_raw[1], g_raw[2]);

  //Apply Filters
  for (int axis = 0; axis < 3; axis++) {
    a_filtered[axis] = accFilters[axis].apply(a_raw[axis]);
    g_filtered[axis] = gyroFilters[axis].apply(g_raw[axis]);
  }

  //Update attitude estimate and extract vertical acceleration
  attitude_estimate.update_estimate(a_filtered, g_filtered, dt, rocketState==ROCKET_PRE); //Only fuse acc if rocket is in pre-flight state //WARNING: ensure that a_raw is in G's and that g_raw is in rad/s, and that dt is in seconds
  newAcc = attitude_estimate.vertical_acceleration_from_acc(a_filtered);
  newAcc *= G_TO_M_S2;
#else
   newAcc = simIn.getInterpolatedAcceleration(simTime);

#endif


  //read h and calculate diffH when a new value of h is ready
  if( (micros() - altimeterMicros) > 23000){ //19.5 ms is the typical pressure read case. Worst case is 22.5 ms. Standby is 0.5 ms
#ifndef SIMULINK_TESTING
    sensors.readAltitude(h_raw);
    h_raw -= h_groundLevel;
#else
    h_raw = simIn.getInterpolatedAltitude(simTime);
#endif

    oldH = h_filtered;
    h_filtered = hFilter.apply(h_raw);
    dt_h = (micros() - altimeterMicros) / 1000000.0f;
    altimeterMicros = micros();
    diffH = (h_filtered - oldH) / dt_h;
  }

  //fill back-calculation list
  backAcc[backCalcIndex] = newAcc;
  backDt[backCalcIndex] = dt;
  backCalcIndex = ((backCalcIndex+1)<=BACK_ACC_LENGTH) ? (backCalcIndex+1) : 0; 
}
inline void prvIntegrateAccel(){
  if(!backCalcDone){
    for(int i=BACK_ACC_LENGTH-1; i>=0; i--){
      // vel += backAcc[i] * backDt[i];
      intA += backAcc[i] * backDt[i]; //This should fix the mega spike at start?
    }
    backCalcDone = true;
  } else {
    intA = newAcc * dt; // will drift, but accurate over short times
  }

  float fusion_gain = 0.99; // how much we trust accelerometer data
  integralOfAccel += intA;
  // diffH = (h_filtered - oldH) / dt; //moved to prvReadSensors
  vel = fusion_gain * (vel + intA) + (1.0 - fusion_gain) * diffH;

}
inline void prvDoControl(){
  desiredH = getDesired(burnoutTime);
  predictedH = predictAltitude(h_filtered,vel);
  ang = getControl(desiredH, predictedH, dt);
  stepper.setStepsTarget(microStepsFromFlapAngle(ang));
}

