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


#include "StepperInterface.hh"


class SpoofStepper : public StepperInterface {
public:
  SpoofStepper();
  //virtual void stepForTime(unsigned long maxTime, unsigned long startingTime) override;
  virtual void stepOnce(void) override;
};


#endif  //SPOOF_STEPPER_HH
