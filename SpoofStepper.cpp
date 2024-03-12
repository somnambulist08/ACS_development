#include "SpoofStepper.hh"


void SpoofStepper::stepOnce(void) {
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
  return;
}

SpoofStepper::SpoofStepper() {

}
/*
void SpoofStepper::stepForTime(unsigned long maxTime, unsigned long startingTime) {
  moveSteps = stepsTarget - currentStep;
  if (moveSteps == 0) return;
  unsigned long stepsAvail = (maxTime * RPM * MOTOR_STEPS * MICROSTEPS) / (1000000 * 60);
  if (moveSteps < 0) stepsAvail = -stepsAvail;
  direction = int(!(moveSteps <= 0));
  //Teleports flaps - if we could get there in the maxTime, go there.
  if (abs(moveSteps) < abs(stepsAvail))
    currentStep = currentStep + moveSteps;
  else
    currentStep = currentStep + stepsAvail;
  return;
}*/
