#include "SerialSpoofStepper.hh"
#include <Arduino.h>
#include <math.h>

SerialSpoofStepper::SerialSpoofStepper(){
  direction = 0;
  stepsTarget = 0;
  currentStep = 0;
}

void SerialSpoofStepper::stepForTime(unsigned long maxTime, unsigned long startingTime){
    
}

void SerialSpoofStepper::stepOnce(void){
  int direction = -1;
  if ((currentStep-stepsTarget)== 0) {
    return;
  } else if((currentStep-stepsTarget) > 0 ){
    direction = 0;
  } else {
    direction = 1;
  }
  if (direction) {
    currentStep++;
  } else {
    currentStep--;
  }
  Serial.print("Step ");
  Serial.println(currentStep);
  delay(PULSE_PERIOD_MS);
}

void SerialSpoofStepper::setStepsTarget(int newTarget){
  Serial.print("New Goal: ");
  Serial.println(newTarget);
  stepsTarget = newTarget;
}

int SerialSpoofStepper::microStepsFromFlapAngle(float angle){
  Serial.print("Recieved Request to Convert Angle: ");
  Serial.println(angle);
  angle -= 0.046121116014511f;
  float A = 103.6259f - (32.53459696f * cos(angle) + sqrt(6814.5025f - pow(43.4f +  32.53459696f * sin(angle), 2)));
  float microSteps = ((float)MICROSTEPS) * A / 2.54f * ((float)MOTOR_STEPS);
  Serial.print("Conversion Yielded Needed Microsteps: ");
  Serial.println(microSteps);
  return (int)microSteps;
}