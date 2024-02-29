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


#ifdef DEVELOPMENT

#include "RocketRTOS.hh"
#include "SerialSpoofStepper.hh"
#include "Control.hh"
//#include "InternalSensors.hh"
#include "SDSpoofer.hh"

#define sensorPeriod

SDSpoofer dummySD;
SerialSpoofStepper stepper;

unsigned long oldMicros;
float velocity;
unsigned long oldAccel;


void setup(){
  Serial.begin(115200);
  while(!Serial);

  dummySD.openFile();
  oldMicros = micros();
  velocity = 0;
  oldAccel = 0;

  startRocketRTOS();
}

void stepperTask(){
  stepper.stepOnce();
}

void sensorAndControlTask(){
  float x=4, y=0, z=8;
  //sensor.readAcceleration(x,y,z);
  Serial.print("Acceleration: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);


  float pressure = 80000;
  Serial.print("Pressure: ");
  Serial.println(pressure);

  float altitude = (1 - powf((pressure / 101325), 0.190284)) * 145366.45 * 0.3048;
  Serial.print("Altitude: ");
  Serial.println(altitude);

  float dt = micros() - oldMicros;
  velocity = (dt * (z + oldAccel) / 2.0);
  Serial.print("Velocity: ");
  Serial.println(velocity);

  float flaps = getControl(100, predictAltitude(altitude, velocity), dt);
  Serial.print("Control angle: ");
  Serial.println(flaps);

  //stepper.setStepsTarget(stepper.microStepsFromFlapAngle(flaps));
  stepper.setStepsTarget(1000000);
}

void loggingTask(){
  dummySD.writeLog();
}

#endif //DEVELOPMENT


#ifdef STATE_TEST


#endif //STATE_TEST

#ifdef CONTROL_TEST

#include "RocketRTOS.hh"
//#include "SDLogger.hh"
#include "SDSpoofer.hh"
//#include "SimulinkData.hh"
#include "RealStepper.hh"
#include <mbed.h>

//SDLogger sd;
RealStepper stepper;
SDSpoofer out;

float accel =0;
float vel =0;
float h =0;
float ang =0;
float oldAccel = 0;
float oldMicros;
mbed::Timer tim;

void setup(){
  Serial.begin(115200);
  while(!Serial);
  
  tim.start();
  oldMicros = tim.elapsedTime().count();
  startRocketRTOS();
}

/*
void sensorAndControlTask(){
  
}*/

void stepperTask(){
  stepper.stepOnce();
}

void loggingTask(){
  out.writeLog(accel, vel, h, ang);
}



#endif //CONTROL_TEST

#ifdef FLIGHT

#endif //FLIGHT

