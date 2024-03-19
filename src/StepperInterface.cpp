#include "StepperInterface.hh"
#include <math.h>

const unsigned long pps = (RPM / 60) * MOTOR_STEPS * MICROSTEPS;
const unsigned long pwmicros = 1000000 / (pps);
const unsigned long pw_on = pwmicros / 2;

int microStepsFromFlapAngle(float angle) {
  angle -= 0.046121116014511f;
  float A = 103.6259f - (32.53459696f * cos(angle) + sqrt(6814.5025f - pow(43.4f +  32.53459696f * sin(angle), 2)));
  float microSteps = ((float)MICROSTEPS) * A / 2.54f * 360.0f / 1.8f;
  return (int)microSteps;
}

StepperInterface::StepperInterface() : moveSteps(0), currentStep(0), direction(0), stepsTarget(0) {

}
void StepperInterface::setStepsTarget(int newTarget){
  stepsTarget = newTarget;
}