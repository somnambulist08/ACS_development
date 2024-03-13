/****************************************************************************
* SimulinkStepper.hh 
* Class for providing simulink with feedback by simulating a stepper
****************************************************************************/
#ifndef SIMULINK_STEPPER_HH
#define SIMULINK_STEPPER_HH

#include "StepperInterface.hh"

class SimulinkStepper : public StepperInterface {
public:
  SimulinkStepper();
  virtual void stepOnce(void) override;
};

#endif //SIMULINK_STEPPER