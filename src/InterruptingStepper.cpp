#include "InterruptingStepper.hh"

#include <Arduino.h>
volatile int stepsTargetGlobal = 0;
volatile int currentStepGlobal = 0;
volatile int directionGlobal = 0;
volatile int zeroStepGlobal = 0;
static volatile bool doStep = false;
//volatile bool printStepLow = false;
//volatile bool printStepHigh = false;

IntervalTimer tickerHigh;
IntervalTimer tickerLow;
IntervalTimer tickerOneshot;

InterruptingStepper::InterruptingStepper(){
    stepsTargetGlobal = 0;
    currentStepGlobal = 0;
    directionGlobal = 0;
    doStep = false;
    //printStepLow = false;
    //printStepHigh = false;
}
void InterruptingStepper::start(){
    pinMode(DIRECTION_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    disable(); //start in disabled state //TODO: decide if this is proper

    tickerHigh.priority(TIMER_INTERRUPT_PRIORITY_HIGH);
    tickerLow.priority(TIMER_INTERRUPT_PRIORITY_LOW);
    tickerOneshot.priority(TIMER_INTERRUPT_PRIORITY_EXTRA_HIGH);

    tickerHigh.begin(stepperPullHigh, pwmicros);
    tickerOneshot.begin(startPullLow, pw_on);
}
void InterruptingStepper::setStepsTarget(int newTarget){
    stepsTargetGlobal = newTarget - zeroStepGlobal;
    directionGlobal = (currentStepGlobal < stepsTargetGlobal) ? 1 : 0 ;
    digitalWrite(DIRECTION_PIN, directionGlobal);
}
void InterruptingStepper::setZero(int newZero){
    zeroStepGlobal = newZero;
    setStepsTarget(stepsTargetGlobal);
}
void InterruptingStepper::stepOnce(){
    if(directionGlobal){
        setZero(zeroStepGlobal+STEPS_PER_STEP_ONCE);
    } else {
        setZero(zeroStepGlobal-STEPS_PER_STEP_ONCE);
    }
}
void InterruptingStepper::enable(){
    digitalWrite(ENABLE_PIN, MOTOR_ENABLE);
}
void InterruptingStepper::disable(){
    digitalWrite(ENABLE_PIN, MOTOR_DISABLE);
}

void stepperPullHigh(){
    if(directionGlobal){
        doStep = currentStepGlobal < stepsTargetGlobal;
    } else {
        doStep = currentStepGlobal > stepsTargetGlobal;
    }
    if(doStep){
        //printStepHigh = true;
        //doStep = true;
        //ODR |= 1 << STEP_PIN;
        digitalWriteFast(STEP_PIN, 1);
    }
}
void stepperPullLow(){
    if(doStep){
        //printStepLow = true;
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

