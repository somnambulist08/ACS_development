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
#define CONTROL_TEST
//#define FLIGHT

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
//#include data collector
//#include "SDLogger.hh"
#include "RealStepper.hh"

#define LAUNCH_THRESHOLD_M_S2 10

//we need a new timer because they last 30 minutes before they overflow.
//If we sit on the pad for longer than that then we don't know if we are just barely
//in the new 30 minutes or if we are going to overflow mid-flight. To avoid this,
//we just reset the timer until it is finally time to run. That way, we are garunteed
//to not overflow during launch
mbed::Timer tim;

RealStepper stepper;
//SDLogger sd;

unsigned long oldTimMicros=0;
float newAcc;
float vel=0;
float oldAcc=0;

void setup(){
  tim.start();

  startRocketRTOS();
}

void determineState(){
  while(newAcc < LAUNCH_THRESHOLD_M_S2){
    rocketState = ROCKET_PRE;
  }
  while(newAcc > 0){
    rocketState = ROCKET_LAUNCH;
  }
  while(vel>0){
    rocketState = ROCKET_FREEFALL;
  }
  while(1){ //once you enter recovery state, do not leave
    rocketState = ROCKET_RECOVERY;
  }
}

void sensorAndControl_PRE(){
  tim.reset(); //continue to reset tim until we determine we are in launch mode
  //get data
}
void sensorAndControl_LAUNCH(){
  //get data
  //newAcc = ;

  //integrate acc to get vel
  float dt = (float)(tim.elapsed_time().count() - oldTimMicros);
  vel = (oldAcc + newAcc)/2 * dt;

  //update variables
  oldAcc = newAcc; 
  oldTimMicros = tim.elapsed_time().count();
}
void sensorAndControl_FULL(){
  //get data

  //integrate accel to get vel
  float dt = (float)(tim.elapsed_time().count() - oldTimMicros);
  vel = (oldAcc + newAcc)/2 * dt;

  //update variables
  oldAcc = newAcc; 
  oldTimMicros = tim.elapsed_time().count();

  //set new control steps

}


void logging_RUN(){
  //sd.writeLog();
}
void logging_CLOSE(){
  //sd.closeFile();
}

void stepper_RUN(){
  stepper.stepOnce();
}
void stepper_CLOSE(){
  stepper.setStepsTarget(0);
  while(1){
    stepper.stepOnce();
  }
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

SimulinkFile simIn;
RealStepper stepper;
SDSpoofer out;

float accX=0, accY=0, accZ=0;
float vel=0;
float h=0;
float ang=0;
float oldAccel=0;
float tNow=0;
float tLast=0;

void setup(){
  Serial.begin(115200);
  while(!Serial);
  
  startRocketRTOS();
}


void sensorAndControl_FULL(){
  //get data
  simIn.readAcceleration(accX,accY,accZ);
  simIn.readFrame(tNow);
  simIn.readAltitude(h);

  //integrate
  float dt = tNow - tLast;
  vel = (accZ - oldAccel)/2.0f * dt;
  
  //set control value
  ang = getControl(getDesired(tNow), predictAltitude(h,vel), dt);
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
#include "InternalSensors.hh"
#include "SDLogger.hh"
#include "RealStepper.hh"

//we need a new timer because they last 30 minutes before they overflow.
//If we sit on the pad for longer than that then we don't know if we are just barely
//in the new 30 minutes or if we are going to overflow mid-flight. To avoid this,
//we just reset the timer until it is finally time to run. That way, we are garunteed
//to not overflow during launch
mbed::Timer tim;

RealStepper stepper;
SDLogger sd;

unsigned long oldTimMicros=0;
float vel=0;
float oldAcc=0;
float newAcc=0;

void setup(){
  tim.start();

  startRocketRTOS();
}

void determineState(){
  while(newAcc < LAUNCH_THRESHOLD_M_S2){
    rocketState = ROCKET_PRE;
  }
  while(newAcc > 0){
    rocketState = ROCKET_LAUNCH;
  }
  while(vel>0){
    rocketState = ROCKET_FREEFALL;
  }
  while(1){ //once you enter recovery state, do not leave
    rocketState = ROCKET_RECOVERY;
  }
}

void sensorAndControl_PRE(){
  tim.reset(); //continue to reset tim until we determine we are in launch mode
  //get data
}
void sensorAndControl_LAUNCH(){
  //get data
  //newAcc = ;

  //integrate acc to get vel
  float dt = (float)(tim.elapsed_tim().count() - oldTimMicros);
  vel = (oldAcc + newAcc)/2 * dt;

  //update variables
  oldAcc = newAcc; 
  oldTimMicros = tim.elapsed_time().count();
}
void sensorAndControl_FULL(){
  //get data

  //integrate accel to get vel
  float dt = (float)(tim.elapsed_tim().count() - oldTimMicros);
  vel = (oldAcc + newAcc)/2 * dt;

  //update variables
  oldAcc = newAcc; 
  oldTimMicros = tim.elapsed_time().count();

  //set new control steps
  
}


void logging_RUN(){
  sd.writeLog();
}
void logging_CLOSE(){
  sd.closeFile();
}

void stepper_RUN(){
  stepper.stepOnce();
}
void stepper_CLOSE(){
  stepper.setStepsTarget(0);
  while(1){
    stepper.stepOnce();
  }
}


#endif //FLIGHT

