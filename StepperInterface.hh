/****************************************************************************
* StepperInterface.hh 
* Base class for talking to a real, interrupting, or imagined stepper motor
*
****************************************************************************/
#ifndef STEPPER_INTERFACE_HH
#define STEPPER_INTERFACE_HH

class StepperInterface {
public:
  StepperInterface(){}
  virtual void stepForTime(unsigned long maxTime, unsigned long startingTime)=0;
  virtual void stepOnce(void)=0;
  virtual void setStepsTarget(int newTarget)=0;
  virtual int microStepsFromFlapAngle(float angle)=0;
private:
  int moveSteps;
  int currentStep;
  int direction;
  int stepsTarget;
};

#endif