#include "SimulinkStepper.hh"
#include <Arduino.h>


SimulinkStepper::SimulinkStepper(){

}

void SimulinkStepper::stepOnce(){
    int direction = -1; 
    if ((currentStep-stepsTarget)== 0) {
        return;
    } else if((currentStep-stepsTarget) > 0 ){
        direction = 0;
    } else {
        direction = 1;
    }
    if (direction) {
        Serial.println("+");
        currentStep++;
    } else {
        currentStep--;
        Serial.println("-");
    }
    delayMicroseconds(pps);
}