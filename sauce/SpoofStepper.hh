/******************************************************************************
* SpoofStepper.hh
*
* Fakes the potentiometer behavior for easier simulator testing
* Uses same function names and footprints as StepperStuff.
* 
* 02/19/2024 - Created file1
******************************************************************************/
#ifndef SPOOF_STEPPER_HH
#define SPOOF_STEPPER_HH


#include "StepperStuff.hh"

#define MOTOR_STEPS 200  //steps per rev
#define RPM 750  //speed?
#define MICROSTEPS 8



//extern const unsigned long frame_micros;
extern const float pps;
extern const unsigned long pwmicros;
extern const unsigned long pw_on;



int microStepsFromFlapAngle(float angle);


class SpoofStepper : public StepperStepper {
public:
  SpoofStepper();
  void stepForTime(unsigned long maxTime, unsigned long startingTime);
  void stepOnce(void);
  void setStepsTarget(int newTarget);
private:
  int moveSteps;
  int currentStep;
  int direction;
  int stepsTarget;
};


#endif  //SPOOF_STEPPER_HH
