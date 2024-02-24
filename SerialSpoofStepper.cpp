#include "SerialSpoofStepper.hh"
#include <Arduino.h>
#include <math.h>

SerialSpoofStepper::SerialSpoofStepper(){
  direction = 0;
  stepsTarget = 0;
  currentStep = 0;
  moveSteps = 0;
}

void SerialSpoofStepper::stepForTime(unsigned long maxTime, unsigned long startingTime){
    
}

void SerialSpoofStepper::stepOnce(void){
  int direction = -1;
  if (moveSteps <= 0) {
    direction = 0;
  } else {
    direction = 1;
  }
  if (direction) {
    moveSteps = moveSteps + 1;
    currentStep = currentStep + 1;
  } else {
    moveSteps = moveSteps + 1;
    currentStep = currentStep - 1;
  }
  Serial.print("Step ");
  Serial.println(currentStep);
}

void SerialSpoofStepper::setStepsTarget(int newTarget){
  Serial.print("New Goal: ");
  Serial.println(newTarget);
  stepsTarget = newTarget;
}

int SerialSpoofStepper::microStepsFromFlapAngle(float angle){
  angle -= 0.046121116014511f;
  float A = 103.6259f - (32.53459696f * cos(angle) + sqrt(6814.5025f - pow(43.4f +  32.53459696f * sin(angle), 2)));
  float microSteps = ((float)MICROSTEPS) * A / 2.54f * ((float)MOTOR_STEPS);
  return (int)microSteps;
}

int SerialSpoofStepper::getMoveSteps(){
    return moveSteps;
}