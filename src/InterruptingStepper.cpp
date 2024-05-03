#include "InterruptingStepper.hh"

#include <Arduino.h>

IntervalTimer tickerHigh;
IntervalTimer tickerLow;
IntervalTimer tickerOneshot;

volatile StepperVars stepperVars = {0};

InterruptingStepper::InterruptingStepper(){
    stepperVars.stepsTarget = 0;
    stepperVars.currentStep = 0;
    stepperVars.direction = 0;
    stepperVars.doStep = false;
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
    stepperVars.stepsTarget = newTarget;
    stepperVars.direction = (stepperVars.currentStep < stepperVars.stepsTarget) ? 1 : 0 ;
    digitalWrite(DIRECTION_PIN, stepperVars.direction);
}
void InterruptingStepper::enable(){
    digitalWrite(ENABLE_PIN, MOTOR_ENABLE);
}
void InterruptingStepper::disable(){
    digitalWrite(ENABLE_PIN, MOTOR_DISABLE);
}

void stepperPullHigh(){
    if(stepperVars.direction){
        stepperVars.doStep = stepperVars.currentStep < stepperVars.stepsTarget;
    } else {
        stepperVars.doStep = stepperVars.currentStep > stepperVars.stepsTarget;
    }
    if(stepperVars.doStep){
        //printStepHigh = true;
        //stepperVars.doStep = true;
        //ODR |= 1 << STEP_PIN;
        digitalWriteFast(STEP_PIN, 1);
    }
}
void stepperPullLow(){
    if(stepperVars.doStep){
        //printStepLow = true;
        //ODR &= ~( 1 << STEP_PIN );
        digitalWriteFast(STEP_PIN, 0);
        stepperVars.doStep = false;
        if(stepperVars.direction){
            stepperVars.currentStep++;
        } else {
            stepperVars.currentStep--;
        }
    }
}
void startPullLow(){
    tickerLow.begin(stepperPullLow, pwmicros);
    tickerOneshot.end();
}

