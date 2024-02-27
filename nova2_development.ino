#define JON_ALTERNATIVE_INO //Comment this line out to do things normally. This is only here for when I need to do some testing outside of the main loop //jonse
#ifndef JON_ALTERNATIVE_INO

#include <math.h>
#include <Arduino.h>
#include <SD.h>

#include "InternalSensors.hh"
#include "RealStepper.hh"
#include "Control.hh"


#define frame_sec 0.01

#define G_MET 9.81
#define G_IMP 32.174


//Pins for LED and Buzzer
#define RED 22
#define GREEN 23
#define BLUE 24  
#define BZZT 6

int currentStep = 0;

//IMU measurements- Right handed, Z positive up
float acc_x = 0.0f,
      acc_y = 0.0f,
      acc_z = 0.0f;

float gyro_x = 0.0f,
      gyro_y = 0.0f,
      gyro_z = 0.0f;

float mag_x = 0.0f,
      mag_y = 0.0f,
      mag_z = 0.0f;


//barometer and thermometer
float temp = 0.0f;
float pressure = 0.0f;


//calculated and derived values
unsigned long t_prev = -1;
unsigned long t_now = 0;
unsigned long dt = 0;
float v_x = 0.0f;
float v_y = 0.0f;
float v_z = 0.0f;
float altitude = 0.0f;
float groundPressure = 0.0f;


const unsigned long frame_micros = frame_sec * 1000000;



unsigned long burnout_time = 0;
unsigned long timeSinceBurnout = 0;
float lastStep = 0;
unsigned long timeAtBurnout = 0;
float ang = 0;

//State Transition booleans
bool preFlight = true;
bool burnStart = false;
bool burnEnd = false;
bool hitApogee = false;

float acc_x_t1 = 0.0f;
float aint_safe = 0.0;
float velocity = 0;

RealStepper stepperStepper;


void setup() {
  pinMode(RED, OUTPUT);  //Debug LED and buzzer-always using these
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(BZZT, OUTPUT);
  
  digitalWrite(BLUE, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(BZZT, LOW);
  
  //conditional pins
  #ifdef flap_pin
  pinMode(flap_pin, OUTPUT);
  #endif
  #ifdef STEP
  pinMode(STEP,OUTPUT);
  #endif
  #ifdef DIRECTION
  pinMode(DIRECTION,OUTPUT);
  #endif




  for (int i = 0; i < 3; i++) {
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, HIGH);
    digitalWrite(RED, HIGH); 
    delay(250);
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, LOW);
    digitalWrite(RED, LOW);  //3 Blinks and a quarter second buzz - Startup complete
    delay(250);
  }

  digitalWrite(BZZT, LOW);
  delay(250);
  digitalWrite(BZZT, HIGH);
  t_now = micros();
  Serial.println("Startup done...");
}

void loop() {
  t_prev = t_now;
  t_now = micros();
  dt = (t_now - t_prev); //time step in microseconds
  //get current flight parameters from somewhere
  //querrySimulinkAndTime(&altitude, &acc_x, &t_now);
  if (preFlight) {
    Serial.println("\t Preflight");
    burnStart = (acc_x >= 50.0f);
    preFlight = !burnStart;
  } else if (burnStart && !burnEnd) {
    Serial.println("\t we be burnin");
    float aint = accelerint(acc_x_t1, acc_x, dt/1000000);
    if (isfinite(aint)) {
      velocity += aint;
      aint_safe = aint;
    } else {
      velocity += aint_safe;
    }
    burnEnd = (acc_x < 0.0f);
    burnStart = !burnEnd;
    timeAtBurnout = t_now;
  } else if (!hitApogee) {
    Serial.println("\t control phase");
    float aint = accelerint(acc_x_t1, acc_x, dt);
    if (isfinite(aint)) {
      velocity += aint;
      aint_safe = aint;
    } else {
      velocity += aint_safe;
    }
    timeSinceBurnout = t_now - timeAtBurnout;
    ang = getControl(getDesired(timeSinceBurnout), predictAltitude(altitude, velocity), timeSinceBurnout - lastStep);
    lastStep = timeSinceBurnout;
    //move the flaps or (pretend to)
    stepperStepper.setStepsTarget(microStepsFromFlapAngle(ang));
    stepperStepper.stepForTime(frame_micros, micros());
    hitApogee = (velocity <= 0);
  } else {
    Serial.print('\n');
    Serial.println("Hit Apogee, cleaning up");
    //postApogee Tasks
    //flightData.close();
    //flightData = SD.open(fileName, FILE_WRITE);
    stepperStepper.setStepsTarget(0);
    stepperStepper.stepForTime(1000000 * 5, micros());  //step for up to 5 seconds (is too long, thus is good);
    delay(1000000);
  }

  //readAndSendFlapsToSimulink();
}




#else //ifdef JON_ALTERNATIVE_INO


#include <Arduino.h>
#include <USB/PluggableUSBSerial.h>
#include <mbed.h>
#include "RocketRTOS.hh"


int main(){
  init(); //General Arduino startup tasks
	initVariant(); //BLE Sense specific Startup tasks

//Opens the virtual COM port
#if defined(SERIAL_CDC)
  PluggableUSBD().begin();
  _SerialUSB.begin(115200);
#endif

/********************************
 * === Our Code Begins Here === *
 ********************************/


  Serial.begin(115200);
  while(!Serial);

  while(1){
    Serial.println("Running Custom Main");
  }

  return -1; 
}















#endif //JON_ALTERNATIVE_INO