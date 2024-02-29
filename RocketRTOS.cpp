#include "RocketRTOS.hh"

rtos::Thread sensorAndControlThread(osPriorityHigh, THREADS_STACK_DEPTH);
rtos::Thread loggingThread(osPriorityHigh, THREADS_STACK_DEPTH);

void sensorAndControlCallback();
void loggingCallback();

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
void loop{
    while(1){ //technically, the main.cpp wrapps loop. However, doing it here reduces the number of callbacks
        stepperTask();
    }
}