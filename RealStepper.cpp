#include "RealStepper.hh"
#include <math.h>
#include "Arduino.h"


int microStepsFromFlapAngle(float angle) {
  angle -= 0.046121116014511f;
  float A = 103.6259f - (32.53459696f * cos(angle) + sqrt(6814.5025f - pow(43.4f +  32.53459696f * sin(angle), 2)));
  float microSteps = ((float)MICROSTEPS) * A / 2.54f * 360.0f / 1.8f;
  return (int)microSteps;
}


void RealStepper::setStepsTarget(int newTarget){
  stepsTarget = newTarget;
  return;
}

void RealStepper::stepOnce(void){
  int direction = -1;
  if (moveSteps <= 0) {
    direction = 0;
  } else {
    direction = 1;
  }
  digitalWrite(STEP, HIGH);
  delayMicroseconds(pw_on);
  digitalWrite(STEP, LOW);
  delayMicroseconds(pwmicros - pw_on);
  if (direction) {
    moveSteps = moveSteps + 1;
    currentStep = currentStep + 1;
  } else {
    moveSteps = moveSteps + 1;
    currentStep = currentStep - 1;
  }
  digitalWrite(DIRECTION, direction);
  return;
}

RealStepper::RealStepper(){
  direction = 0;
  stepsTarget = 0;
  currentStep = 0;
  moveSteps = 0;
  return;
}

void RealStepper::stepForTime(unsigned long maxTime, unsigned long startingTime){
  moveSteps = stepsTarget - currentStep;
  //Serial.print("moveSteps=");
  //Serial.println(moveSteps);
  if (moveSteps = 0) return;
  unsigned long prev = 0;
  while ((micros() - startingTime) < maxTime) {

    unsigned long current = micros();
    if ((current - prev >= pwmicros) && moveSteps != 0) {
      prev = current;
      stepOnce();
    }
  }
  return;
}