#include "RocketRTOS.hh"

//Threads at higher priority than main
rtos::Thread sensorAndControlThread(osPriorityRealtime, THREADS_STACK_DEPTH);
rtos::Thread loggingThread(osPriorityHigh, THREADS_STACK_DEPTH);
//Threads at lower priority than main
rtos::Thread stepperThread(osPriorityLow, THREADS_STACK_DEPTH);

RocketState_t rocketState = ROCKET_BOOT;

void sensorAndControlCallback();
void loggingCallback();
void stepperCallback();

//This file provides a definition for deterimining state, but if any other file wants to define it they may
void determineState() __attribute__((weak));

//Tasks form this file should only be used if they are not defined in any other file
void stepper_RUN() __attribute__((weak)); 
void stepper_CLOSE() __attribute__((weak));
void stepper_IDLE() __attribute__((weak));

void sensorAndControl_PRE() __attribute__((weak));
void sensorAndControl_LAUNCH() __attribute__((weak));
void sensorAndControl_FULL() __attribute__((weak));
void sensorAndControl_IDLE() __attribute__((weak));

void logging_RUN() __attribute__((weak));
void logging_CLOSE() __attribute__((weak));
void logging_IDLE() __attribute__((weak));

void startRocketRTOS(){
    rocketState = ROCKET_PRE;
    sensorAndControlThread.start(sensorAndControlCallback);
    loggingThread.start(loggingCallback);
    stepperThread.start(stepperCallback);
}   

void sensorAndControlCallback(){
    while(1){
        switch(rocketState){
            case(ROCKET_PRE):
                sensorAndControl_PRE();
                break;
            case(ROCKET_LAUNCH):
                sensorAndControl_LAUNCH();
                break;
            case(ROCKET_FREEFALL):
                sensorAndControl_FULL();
                break;
            default:
                sensorAndControl_IDLE();
                break;
        }
        delay(SENSOR_AND_CONTROL_DELAY_MS);
    }
}

void loggingCallback(){
    while(1){
       switch(rocketState){
            case(ROCKET_PRE):
            case(ROCKET_LAUNCH):
            case(ROCKET_FREEFALL):
                logging_RUN();
                break;
            case(ROCKET_RECOVERY):
                logging_CLOSE();
                break;
            default:
                logging_IDLE();
                break;
        }
        delay(LOGGING_DELAY_MS);
    }
}

void stepperCallback(){
    while(1){
        switch(rocketState){
            case(ROCKET_FREEFALL):
                stepper_RUN();
                break;
            case(ROCKET_RECOVERY):
                stepper_CLOSE();
                break;
            default:
                stepper_IDLE();
                break;
        }
        //No delay in lowest-priority thread
    }
}

//main thread tracks the state
void loop(){
    while(1){
        determineState();
        delay(STATE_CHECKING_DELAY_MS); //make an opening for the stepper
    }
}

//if nobody else defines determinState(), the state is always ROCKET_FREEFALL
void determineState() {
    rocketState = ROCKET_FREEFALL;
}



//Default Task definitions
void stepper_RUN() {stepper_IDLE();}
void stepper_CLOSE() {stepper_IDLE();}
void stepper_IDLE() {yield();}

void sensorAndControl_PRE() {sensorAndControl_IDLE();}
void sensorAndControl_LAUNCH() {sensorAndControl_IDLE();}
void sensorAndControl_FULL() {sensorAndControl_IDLE();}
void sensorAndControl_IDLE() {yield();}

void logging_RUN() {logging_IDLE();}
void logging_CLOSE() {logging_IDLE();}
void logging_IDLE() {yield();}

