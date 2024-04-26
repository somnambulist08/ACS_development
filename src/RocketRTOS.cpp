#include "RocketRTOS.hh"


#ifdef ROCKETRTOS_MBED
//Threads at higher priority than main
rtos::Thread sensorAndControlThread(osPriorityRealtime, THREADS_STACK_DEPTH);
rtos::Thread loggingThread(osPriorityHigh, THREADS_STACK_DEPTH);
//Threads at lower priority than main
rtos::Thread buzzerThread(osPriorityBelowNormal, THREADS_STACK_DEPTH);
rtos::Thread stepperThread(osPriorityLow, THREADS_STACK_DEPTH);
#endif //ROCKETRTOS_MBED
#ifdef ROCKETRTOS_FREERTOS
//Threads at higher priority than main
TaskHandle_t sensorAndControlThread;
TaskHandle_t loggingThread;
//The "Main Thread" (except FreeRTOS doesn't have a main thread. this is just the middle priority thread)
TaskHandle_t stateCheckingThread;
//Threads at lower priority than main
TaskHandle_t buzzerThread;
TaskHandle_t stepperThread;
#endif //ROCKETRTOS_FREERTOS

RocketState_t rocketState = ROCKET_BOOT;

#ifdef ROCKETRTOS_FREERTOS
void sensorAndControlCallback(void *in);
void loggingCallback(void *in);
void stepperCallback(void *in);
void buzzerCallback(void *in);
void determineStateCallback(void *in);
#else //if not ROCKETRTOS_FREERTOS
void sensorAndControlCallback();
void loggingCallback();
void stepperCallback();
void buzzerCallback();
void determineStateCallback();
#endif //ROCKETRTOS_FREERTOS


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

void buzz_PRE() __attribute__((weak));
void buzz_POST() __attribute__((weak));
void buzz_IDLE() __attribute__((weak));

void startRocketRTOS(){
    rocketState = ROCKET_PRE;

    #ifdef ROCKETRTOS_MBED
        sensorAndControlThread.start(sensorAndControlCallback);
        loggingThread.start(loggingCallback);
        stepperThread.start(stepperCallback);
        buzzerThread.start(buzzerCallback);
    #endif

    #ifdef ROCKETRTOS_FREERTOS
        Serial.println("In startRocketRTOS");

        BaseType_t status;
        status = xTaskCreate(sensorAndControlCallback, "Sensor&Control", THREADS_STACK_DEPTH, NULL, 7, &sensorAndControlThread);
        if(status == pdFAIL) while(1){Serial.println("Can't Start Sensor&Control");}

        status = xTaskCreate(loggingCallback, "Logging", THREADS_STACK_DEPTH, NULL, 6, &loggingThread);
        if(status == pdFAIL) while(1){Serial.println("Can't Start Logging");}

        status = xTaskCreate(determineStateCallback, "StateMachine", THREADS_STACK_DEPTH, NULL, 5, &stateCheckingThread);
        if(status == pdFAIL) while(1){Serial.println("Can't Start StateMachine");}

        status = xTaskCreate(buzzerCallback, "Buzzer", THREADS_STACK_DEPTH, NULL, 4, &buzzerThread);
        if(status == pdFAIL) while(1){Serial.println("Can't Start Buzzer");}

        status = xTaskCreate(stepperCallback, "Stepper", THREADS_STACK_DEPTH, NULL, 3, &stepperThread);
        if(status == pdFAIL) while(1){Serial.println("Can't Start Stepper");}


        Serial.println("Starting Scheduler");
        vTaskStartScheduler();
        Serial.println("Scheduler Aborted");
    #endif
    
    #ifdef ROCKETRTOS_TEENSYTHREADS

    #endif

    #ifdef ROCKETRTOS_NONE
        if(Serial) Serial.println("RocketRTOS is currently running in fixed-loop mode. Did you implement stepper and buzzer interrupts?");
    #endif


}   

#ifdef ROCKETRTOS_FREERTOS
//main thread should never execute
void loop() __attribute__((weak)); //to let me define it when I'm not using the RTOS because Arduino compiles and links this file even if I don't include it
void loop(){
    while(1){Serial.println("Scheduler Aborted????");} //if we get here VERY BAD things have happened
}

void determineStateCallback(void *in){
    while(1){
        // Serial.println("DetermineState");
        determineState();
        vTaskDelay(STATE_CHECKING_DELAY_MS/portTICK_PERIOD_MS);
    }
}
#else //If not ROCKETRTOS_FREERTOS
    #ifndef ROCKETRTOS_NONE
    //main thread tracks the state
    void loop() __attribute__((weak)); //to let me define it when I'm not using the RTOS because Arduino compiles and links this file even if I don't include it
    void loop(){
        while(1){
            determineState();
            delay(STATE_CHECKING_DELAY_MS); //make an opening for the stepper
        }
    }
    #else //if def ROCKETRTOS_NONE
    void loop(){
        static unsigned long oldMicros = 0;
        if((micros() - oldMicros) >= 10000){
            determineState();
            sensorAndControlCallback();
            loggingCallback();
            stepperCallback();
            buzzerCallback();
            oldMicros = micros();
        }
    }
    #endif //ROCKETRTOS_NONE
#endif //ROCKETRTOS_FREERTOS


#ifdef ROCKETRTOS_FREERTOS
void sensorAndControlCallback(void *in){
#else
void sensorAndControlCallback(){
#endif //ROCKETRTOS_FREERTOS
#ifndef ROCKETRTOS_NONE
    while(1){
#endif //ROCKETRTOS_NON
        // Serial.println("SensorAndControl");
        switch(rocketState){
            case(ROCKET_PRE):
            case(ROCKET_RECOVERY):
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
        
        #ifdef ROCKETRTOS_FREERTOS
            vTaskDelay(SENSOR_AND_CONTROL_DELAY_MS/portTICK_PERIOD_MS);
        #else
            #ifndef ROCKETRTOS_NONE
                delay(SENSOR_AND_CONTROL_DELAY_MS);
            #endif //ROCKETRTOS_NONE
        #endif
#ifndef ROCKETRTOS_NONE
    }
#endif //ROCKETRTOS_NON
}

#ifdef ROCKETRTOS_FREERTOS
void loggingCallback(void *in){
#else
void loggingCallback(){
#endif //ROCKETRTOS_FREERTOS
#ifndef ROCKETRTOS_NONE
    while(1){
#endif //ROCKETRTOS_NON
        // Serial.println("Logging");
        switch(rocketState){
            case(ROCKET_PRE):
            case(ROCKET_LAUNCH):
            case(ROCKET_FREEFALL):
            case(ROCKET_RECOVERY):
                logging_RUN();
                break;
            // case(ROCKET_RECOVERY):
            //     logging_CLOSE();
            //     break;
            default:
                logging_IDLE();
                break;
        }
        #ifdef ROCKETRTOS_FREERTOS
            vTaskDelay(LOGGING_DELAY_MS/portTICK_PERIOD_MS);
        #else
            #ifndef ROCKETRTOS_NONE
                delay(LOGGING_DELAY_MS);
            #endif //ROCKETRTOS_NONE
        #endif
#ifndef ROCKETRTOS_NONE
    }
#endif //ROCKETRTOS_NON
}
#ifdef ROCKETRTOS_FREERTOS
void stepperCallback(void *in){
#else
void stepperCallback(){
#endif //ROCKETRTOS_FREERTOS
#ifndef ROCKETRTOS_NONE
    while(1){
#endif //ROCKETRTOS_NON
        // Serial.println("Stepper");
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
#ifndef ROCKETRTOS_NONE
    }
#endif //ROCKETRTOS_NON
}


#ifdef ROCKETRTOS_FREERTOS
void buzzerCallback(void *in){
#else
void buzzerCallback(){
#endif //ROCKETRTOS_FREERTOS
#ifndef ROCKETRTOS_NONE
    while(1){
#endif //ROCKETRTOS_NON
        // Serial.println("buzzer");
        switch(rocketState){
            case(ROCKET_PRE):
                buzz_PRE();
                break;
            case(ROCKET_RECOVERY):
                buzz_POST();
                break;
            default:
                buzz_IDLE();
                break;
        }
        #ifdef ROCKETRTOS_FREERTOS
            vTaskDelay(BUZZ_DELAY_MS/portTICK_PERIOD_MS);
        #else
            #ifndef ROCKETRTOS_NONE
                delay(BUZZ_DELAY_MS);
            #endif //ROCKETRTOS_NONE
        #endif
#ifndef ROCKETRTOS_NONE
    }
#endif //ROCKETRTOS_NON
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

void buzz_PRE() {buzz_IDLE();}
void buzz_POST() {buzz_IDLE();}
void buzz_IDLE() {yield();}
