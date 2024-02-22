/******************************************************************************
* RealStepper.hh
*
* Functions and constants to make our motor move to the correct location
* Derived from StepperStuff and renamed to clarify: works with actual motor
*
* 02/15/2024 - Created file
******************************************************************************/
#ifndef REAL_STEPPER_HH
#define REAL_STEPPER_HH


#define MOTOR_STEPS 200 //steps per rev
#define RPM 750  
#define MICROSTEPS 8 //set by encoder but needed for math

//pins.
#define STEP 9
#define DIRECTION 8

extern const unsigned long frame_micros;
const unsigned long pps = (RPM / 60) * MOTOR_STEPS * MICROSTEPS;
const unsigned long pwmicros = 1000000 / (pps);
const unsigned long pw_on = pwmicros / 2;

int microStepsFromFlapAngle(float angle);


class RealStepper{
public:
  RealStepper();
  void stepForTime(unsigned long maxTime, unsigned long startingTime);
  void stepOnce(void);
  void setStepsTarget(int newTarget);
private:
  int moveSteps;
  int currentStep;
  int direction;
  int stepsTarget;
};


#endif //REAL_STEPPER_HH
