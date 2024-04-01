/******************************************************************************
 * InterruptingStepper.hh
 * Implements the functionality to have the stepper as an interrupt rather than
 * a task
******************************************************************************/
#ifndef INTERRUPTING_STEPPER_HH
#define INTERRUPTING_STEPPER_HH

#include <IntervalTimer.h>
#include <StepperInterface.hh>

#define DIRECTION_PIN 7
#define STEP_PIN 6

IntervalTimer tickerHigh;
IntervalTimer tickerLow;
IntervalTimer tickerOneshot;

void stepperPullHigh();
void stepperPullLow();
void startPullLow();

#define TIMER_INTERRUPT_PRIORITY_LOW 127
#define TIMER_INTERRUPT_PRIORITY_HIGH 126
#define TIMER_INTERRUPT_PRIORITY_EXTRA_HIGH 125

//duplicates of these variables so they'll run faster in interrupt context.
volatile int currentStepGlobal;
volatile int stepsTargetGlobal;
volatile int directionGlobal;
volatile bool doStep;

class InterruptingStepper{ //: public StepperInterface {
public:
    InterruptingStepper();
    void stepOnce(void);
    void setStepsTarget(int newTarget);
};


#endif