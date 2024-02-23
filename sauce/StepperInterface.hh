/****************************************************************************
* StepperInterface.h 
* Base class for talking to a real, interrupting, or imagined stepper motor
*
****************************************************************************/
#ifndef STEPPER_INTERFACE_HH
#define STEPPER_INTERFACE_HH

class StepperInterface {
public:
  StepperInterface();
  virtual void stepForTime(unsigned long maxTime, unsigned long startingTime);
  virtual void stepOnce(void);
  virtual void setStepsTarget(int newTarget);
  virtual int microStepsFromFlapAngle(float angle);
private:
  int moveSteps;
  int currentStep;
  int direction;
  int stepsTarget;
};

#endif