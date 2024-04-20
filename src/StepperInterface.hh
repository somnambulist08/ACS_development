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

#define FLAP_LEVER_ARM 32.580300796647045f
#define FLAP_LEVER_ANGLE 0.049743810028906f
#define L_END 102.8899540467910f
#define DELTA_R 44.81012f
#define HINGE_SQUARED 6814.5025f //(82.55f * 82.55f)


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