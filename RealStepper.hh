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

#include "StepperInterface.hh"

//pins.
#define STEP 9
#define DIRECTION 8

class RealStepper : public StepperInterface{
public:
  RealStepper();
  virtual void stepOnce(void) override;
};


#endif //REAL_STEPPER_HH
