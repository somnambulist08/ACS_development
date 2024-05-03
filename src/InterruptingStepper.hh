/******************************************************************************
 * InterruptingStepper.hh
 * Implements the functionality to have the stepper as an interrupt rather than
 * a task
******************************************************************************/
#ifndef INTERRUPTING_STEPPER_HH
#define INTERRUPTING_STEPPER_HH

#include <IntervalTimer.h>
#include <StepperInterface.hh>

#define ENABLE_PIN 7
#define DIRECTION_PIN 8
#define STEP_PIN 9

#define MOTOR_ENABLE 0
#define MOTOR_DISABLE 1

#define STEPS_PER_STEP_ONCE 5

extern IntervalTimer tickerHigh;
extern IntervalTimer tickerLow;
extern IntervalTimer tickerOneshot;

void stepperPullHigh();
void stepperPullLow();
void startPullLow();

#define TIMER_INTERRUPT_PRIORITY_LOW 127
#define TIMER_INTERRUPT_PRIORITY_HIGH 126
#define TIMER_INTERRUPT_PRIORITY_EXTRA_HIGH 125

struct StepperVars {
    int currentStep;
    int stepsTarget;
    int direction;
    bool doStep;
};
extern volatile StepperVars stepperVars;

class InterruptingStepper{ //: public StepperInterface {
public:
    InterruptingStepper();
    void start();
    void setStepsTarget(int newTarget);
    void enable();
    void disable();
};


#endif