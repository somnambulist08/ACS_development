#include "RealStepper.hh"
#include <Arduino.h>

void RealStepper::stepOnce(void){
  int direction = -1;
  if ((currentStep-stepsTarget)== 0) {
    return;
  } else if((currentStep-stepsTarget) > 0 ){
    direction = 0;
  } else {
    direction = 1;
  }
  digitalWrite(DIRECTION, direction);
  digitalWrite(STEP, HIGH);
  delayMicroseconds(pw_on);
  digitalWrite(STEP, LOW);
  delayMicroseconds(pwmicros - pw_on);
  if (direction) {
    currentStep++;
  } else {
    currentStep--;
  }
}

RealStepper::RealStepper(){

}