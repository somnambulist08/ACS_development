#include "RocketRTOS.hh"

/*
void threadHelloWorld(void){
    while(1){
        Serial.println("I'm in a Thread!");
    }
}
*/


void startRocketRTOS(){

#ifdef LOOP_CHECKIN_TEST
    Scheduler.startLoop(loop2);
    Scheduler.startLoop(loop3);
#endif //LOOP_CHECKIN_TEST

#ifdef DUMMY_FUNCTIONS
    Scheduler.startLoop(getSensorsAndDoControlTask);
    Scheduler.startLoop(logToFileTask);
    Scheduler.startLoop(moveStepperTask);
#endif //DUMMY_FUNCTIONS


}


#ifdef DUMMY_FUNCTIONS
void loop(){

    yield(); // basically just means the main loop should be ignored
}


void getSensorsAndDoControlTask(){
    
    for(volatile unsigned int i=0; i<1000; i++); //waste thousands of clock cycles
    Serial.println("Sensor Data Aquired");

    delay(SENSOR_PERIOD_MS);
}
void logToFileTask(){

    for(volatile unsigned int i=0; i<100000; i++); //waste MORE cycles!
    Serial.println("Writing to File Complete");

    delay(FILE_PERIOD_MS);
}
void moveStepperTask(){

    for(volatile unsigned int i=0; i<100; i++); //waste a few cycles
    Serial.println("Stepper Moved");

    delay(STEPPER_PERIOD_MS);
}
#endif

#ifdef LOOP_CHECKIN_TEST
void loop2(){
    Serial.println("Loop2, checking in");
    delay(2000);
}

void loop3(){
    Serial.println("Loop3, checking in");
    delay(3000);
}

//this is the main loop. Will this even work or compile? //it sure does
void loop(){
  Serial.println("Hiiii! I'm the main thread!");
  delay(1000); 
}
#endif //LOOP_CHECKIN_TEST