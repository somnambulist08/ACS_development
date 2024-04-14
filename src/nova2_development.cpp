#include <Arduino.h>
#include "RocketRTOS.hh"
#include "InterruptingStepper.hh"
#include "SDLogger.hh"
#include "Control.hh"
#include "QuickSilver.hh"
#include "Filter.hh"
#include "StateMachine.hh"
#include "ExternalSensors.hh"
//#include "SimulinkData.hh"
// #include "SDSpoofer.hh"
#include <IntervalTimer.h>
#include <climits>



#define BUZZ_PIN 6
// #define BUZZ_PIN 38 //To disable the buzzer so I can code in public :)
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
SDLogger sd;
// SDSpoofer sd;
// SimulinkFile simIn;
StateMachine state;
ExternalSensors sensors;

inline void prvReadSensors();
inline void prvIntegrateAccel();
inline void prvDoControl();

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

float integralOfAccel = 0;

int h_resetCounter=0;

float a_raw[3] = {0.0f, 0.0f, 0.0f};
float a_filtered[3] = {0.0f, 0.0f, 0.0f};
float g_raw[3] = {0.0f, 0.0f, 0.0f};
float g_filtered[3] = {0.0f, 0.0f, 0.0f};
float dt = 0.001;
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
  // Serial.begin(115200);
  // while(!Serial);
  // Serial.println("Serial Connected");

  // initialize the filteres
  for (int axis = 0; axis < 3; axis++) {
    gyroFilters[axis].init(10.0, dt); // dt fed in here should be the rate at which we read new acc data
    accFilters[axis].init(5.0, dt);
  }
  hFilter.init(1.0, dt_h);

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

  sd.openFile("Acc, Vel, h_raw, h_filtered, h_ground, Ang, simT, burnoutT, State, DesiredH, PredictedH, intA, diffH, IntegralOfA, dt, 1/dt, dt_h, 1/dt_h, a_raw[0], a_raw[1], a_raw[2], a_filtered[0], a_filtered[1], a_filtered[2], g_raw[0], g_raw[1], g_raw[2], g_filtered[0], g_filtered[1], g_filtered[2], grav[0], grav[1], grav[2]");
  // sd.openFile("AX, AY, AZ, GX, GY, GZ");

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
  //Serial.println("log run");
  //sd.writeLog(newAcc, vel, h, ang, simTime, burnoutTime, rocketState);
  // sd.writeLog(String("A: ") + String(newAcc) + String(", V:") + String(vel) + String(", H: ") + String(h) + String(", Ang:") + String(ang) + String(", simTime:") + String(simTime) + String(", burnoutTime") + String(burnoutTime) + String(", State:") + String(rocketState) + String(", H_d:") + String(desiredH) + String(", H_p:") + String(predictedH));
  String log = String(newAcc) + String(", ") + String(vel) + String(", ") + String(h_raw) + String(", ") + String(h_filtered) + String(", ") + String(h_groundLevel) + String(", ") 
            + String(ang) + String(", ") + String(simTime) + String(", ") + String(burnoutTime) + String(", ") + String(rocketState) + String(", ") 
            + String(desiredH) + String(", ") + String(predictedH) + String(", ") + String(intA) + String(", ") + String(diffH) + String(", ") + String(integralOfAccel) + String(", ")
            + String(dt) + String(", ") + String((1.0f/dt)) + String(", ")  + String(dt_h) + String(", ") + String(1.0f/dt_h) + String(", ")
            + String(a_raw[0]) + String(", ") + String(a_raw[1]) + String(", ") + String(a_raw[2]) + String(", ") 
            + String(a_filtered[0]) + String(", ") + String(a_filtered[1]) + String(", ") + String(a_filtered[2]) + String(", ") 
            + String(g_raw[0]) + String(", ") + String(g_raw[1]) + String(", ") + String(g_raw[2]) + String(", ") 
            + String(g_filtered[0]) + String(", ") + String(g_filtered[1]) + String(", ") + String(g_filtered[2]) + String(", ")
            + String(attitude_estimate.getGravityVector()[0]) + String(", ") + String(attitude_estimate.getGravityVector()[1]) + String(", ") + String(attitude_estimate.getGravityVector()[2]);
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
  // String log = String("VertAcc: ") + String(newAcc);
  // String log = String(a_raw[0]) + String(", ") + String(a_raw[1]) + String(", ") + String(a_raw[2]) + String(", ") + String(g_raw[0]) + String(", ") + String(g_raw[1]) + String(", ") + String(g_raw[2]);
  // String log = String("Vel: ") + String(vel) + String(", IntA: ") + String(intA) + String(", DiffH: ") + String(diffH);
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
  // stepper.disable();
  digitalWriteFast(ENABLE_PIN, MOTOR_DISABLE);
  // digitalWriteFast(ENABLE_PIN, MOTOR_ENABLE);  
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
  sensors.readAcceleration(a_raw[0], a_raw[1], a_raw[2]);
  sensors.readGyroscope(g_raw[0], g_raw[1], g_raw[2]);
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
    dt_h = (micros() - altimeterMicros) / 1000000.0f;
    altimeterMicros = micros();
    diffH = (h_filtered - oldH) / dt_h;
  }

  simTime = ((float)( micros() - testStartMicros)) / 1000000.0f;
  burnoutTime = ((float)( micros() - burnoutMicros )) / 1000000.0f;
  // h = simIn.getInterpolatedAltitude(simTime);
  // newAcc = simIn.getInterpolatedAcceleration(simTime);
  
  microsNow = micros();
  //calculate dt but catch any overflow error
  if(microsNow > microsOld){
    deltaMicros = (microsNow - microsOld);
  } else {
    deltaMicros = (ULONG_MAX - microsOld) - microsNow;
  }

  dt = ((float)deltaMicros) / 1000000.0f;


  //Attitude Determination
  //Using Raw
  // attitude_estimate.update_estimate(a_raw, g_raw, dt); // TODO ensure that a_raw is in G's and that g_raw is in rad/s, and that dt is in seconds
  // float a_m_s[3] = {a_raw[0] * G_TO_M_S2, a_raw[1] * G_TO_M_S2, a_raw[2] * G_TO_M_S2};
  // newAcc = attitude_estimate.vertical_acceleration_from_acc(a_m_s); // TODO a_m_s here should be in m/^2, ensure that it is
  //Using FIltered
  attitude_estimate.update_estimate(a_filtered, g_filtered, dt, rocketState==ROCKET_PRE); //Only fuse acc if rocket is in pre-flight state //WARNING: ensure that a_raw is in G's and that g_raw is in rad/s, and that dt is in seconds
  newAcc = attitude_estimate.vertical_acceleration_from_acc(a_filtered);
  newAcc *= G_TO_M_S2;

  //update old variables
  microsOld = microsNow;

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
  // ang = getControl(getDesired(burnoutTime), predictAltitude(h,vel), dt);
  ang = getControl(desiredH, predictedH, dt);
  stepper.setStepsTarget(microStepsFromFlapAngle(ang));
}

