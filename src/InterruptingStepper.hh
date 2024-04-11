/******************************************************************************
 * InterruptingStepper.hh
 * Implements the functionality to have the stepper as an interrupt rather than
 * a task
******************************************************************************/
#ifndef INTERRUPTING_STEPPER_HH
#define INTERRUPTING_STEPPER_HH

#include <IntervalTimer.h>
#include <StepperInterface.hh>

#define DIRECTION_PIN 10
#define STEP_PIN 11
#define ENABLE_PIN 9

extern IntervalTimer tickerHigh;
extern IntervalTimer tickerLow;
extern IntervalTimer tickerOneshot;

void stepperPullHigh();
void stepperPullLow();
void startPullLow();

#define TIMER_INTERRUPT_PRIORITY_LOW 127
#define TIMER_INTERRUPT_PRIORITY_HIGH 126
#define TIMER_INTERRUPT_PRIORITY_EXTRA_HIGH 125

//duplicates of these variables so they'll run faster in interrupt context.
extern volatile int currentStepGlobal;
extern volatile int stepsTargetGlobal;
extern volatile int directionGlobal;
//extern volatile bool printStepHigh;    
//extern volatile bool printStepLow;

class InterruptingStepper{ //: public StepperInterface {
public:
    InterruptingStepper();
    void start();
    void stepOnce(void);
    void setStepsTarget(int newTarget);
    void enable();
    void disable();
};


#endif