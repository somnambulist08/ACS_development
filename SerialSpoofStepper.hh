/******************************************************************************
 * SerialSpoofStepper.hh
 * 
 * A Spoof Stepper alternative that prints to serial rather than steps.
 * Usefull for debugging in RocketRTOS
 * 
 * 02/24/24 - Created File
 * 
******************************************************************************/
#ifndef SERIAL_SPOOF_STEPPER_HH
#define SERIAL_SPOOF_STEPPER_HH


#include "StepperInterface.hh"

#define MOTOR_STEPS 200
#define RPM
#define MICROSTEPS 8
const unsigned long PPS = (RPM / 60) * MOTOR_STEPS * MICROSTEPS;
const unsigned long PULSE_PERIOD_MS = (unsigned long)(1000.0f/((float)PPS));

int microStepsFromFlapAngle(float angle);


class SerialSpoofStepper : public StepperInterface {
public:
  StepperInterface();
  void stepForTime(unsigned long maxTime, unsigned long startingTime);
  void stepOnce(void);
  void setStepsTarget(int newTarget);
  int microStepsFromFlapAngle(float angle);

  int getMoveSteps();
private:
  int moveSteps;
  int currentStep;
  int direction;
  int stepsTarget;
};




#endif //SERIAL_SPOOF_STEPPER_HH
