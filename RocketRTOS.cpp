#include "RocketRTOS.hh"

/*
void threadHelloWorld(void){
    while(1){
        Serial.println("I'm in a Thread!");
    }
}
*/
void startRocketRTOS(){
    Scheduler.startLoop(loop2);
    Scheduler.startLoop(loop3);
}

void loop2(){
    Serial.println("Loop2, checking in");
    delay(1000);
}

void loop3(){
    Serial.println("Loop3, checking in");
    delay(1000);
}
