/****************************************************************************
* StepperInterface.hh 
* Base class for talking to a real, interrupting, or imagined stepper motor
*
****************************************************************************/
#ifndef STEPPER_INTERFACE_HH
#define STEPPER_INTERFACE_HH

#define MOTOR_STEPS 200 //steps per rev
#define RPM 750  
#define MICROSTEPS 8 //set by encoder but needed for math

//extern const unsigned long frame_micros;
extern const unsigned long pps;
extern const unsigned long pwmicros;
extern const unsigned long pw_on;

int microStepsFromFlapAngle(float angle); 

class StepperInterface {
public:
  StepperInterface();
  //virtual void stepForTime(unsigned long maxTime, unsigned long startingTime)=0;
  virtual void stepOnce(void)=0;
  void setStepsTarget(int newTarget);
protected:
  int moveSteps;
  int currentStep;
  int direction;
  int stepsTarget;
};

#endif