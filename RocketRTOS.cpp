#include "RocketRTOS.hh"

rtos::Thread sensorAndControlThread(osPriorityHigh, THREADS_STACK_DEPTH);
rtos::Thread loggingThread(osPriorityHigh, THREADS_STACK_DEPTH);

void sensorAndControlCallback();
void loggingCallback();

//Tasks form this file should only be used if they are not defined in any other file
void stepperTask() __attribute__((weak));
void sensorAndControlTask() __attribute__((weak));
void loggingTask() __attribute__((weak));

void startRocketRTOS(){
    sensorAndControlThread.start(sensorAndControlCallback);
    loggingThread.start(loggingCallback);

}   

void sensorAndControlCallback(){
    while(1){
        sensorAndControlTask();
        delay(SENSOR_AND_CONTROL_DELAY_MS);
    }
}

void loggingCallback(){
    while(1){
        loggingTask();
        delay(LOGGING_DELAY_MS);
    }
}

//main loop at a lower priority than the others
void loop(){
    while(1){ //technically, the main.cpp wrapps loop. However, doing it here reduces the number of callbacks
        stepperTask();
    }
}



//Default Task definitions
void stepperTask() {while(1) yield();}
void sensorAndControlTask() {while(1) yield();}
void loggingTask() {while(1) yield();}
