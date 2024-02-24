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
    delay(2000);
}

void loop3(){
    Serial.println("Loop3, checking in");
    delay(3000);
}


//this is the main loop. Will this even work or compile?
void loop(){
  Serial.println("Hiiii! I'm the main thread!");
  delay(1000); 
}