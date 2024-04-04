#include "InterruptingStepper.hh"

#include <Arduino.h>

InterruptingStepper::InterruptingStepper(){
    stepsTargetGlobal = 0;
    currentStepGlobal = 0;
    directionGlobal = 0;
    doStep = false;

    tickerHigh.priority(TIMER_INTERRUPT_PRIORITY_HIGH);
    tickerLow.priority(TIMER_INTERRUPT_PRIORITY_LOW);
    tickerOneshot.priority(TIMER_INTERRUPT_PRIORITY_EXTRA_HIGH);

    tickerHigh.begin(stepperPullHigh, pwmicros);
    tickerOneshot.begin(startPullLow, pw_on);
}
void InterruptingStepper::setStepsTarget(int newTarget){
    stepsTargetGlobal = newTarget;
    directionGlobal = (currentStepGlobal - stepsTargetGlobal) ? 1 : 0 ;
    digitalWrite(DIRECTION_PIN, directionGlobal);
}
void InterruptingStepper::stepOnce(){}

void stepperPullHigh(){
    if(directionGlobal){
        doStep = currentStepGlobal < stepsTargetGlobal;
    } else {
        doStep = currentStepGlobal > stepsTargetGlobal;
    }
    if(doStep){
        doStep = true;
        //ODR |= 1 << STEP_PIN;
        digitalWriteFast(STEP_PIN, 1);
    }
}
void stepperPullLow(){
    if(doStep){
        //ODR &= ~( 1 << STEP_PIN );
        digitalWriteFast(STEP_PIN, 0);
        doStep = false;
        if(directionGlobal){
            currentStepGlobal++;
        } else {
            currentStepGlobal--;
        }
    }
}
void startPullLow(){
    tickerLow.begin(stepperPullLow, pwmicros);
    tickerOneshot.end();
}

