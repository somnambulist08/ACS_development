/******************************************************************************
 * RocketRTOS.hh
 * 
 * Includes the needed functions to start working with RTOS functions and
 * concepts. Some are taken from the Mbed RTOS library and othes are custom
 * made. 
 * 
 * 02/23/2024 - Created File
*****************************************************************************/
#ifndef ROCKET_RTOS_HH
#define ROCKET_RTOS_HH


/*****************************************************************************
 * Choose one define!
*****************************************************************************/
//#define ROCKETRTOS_MBED
//#define ROCKETRTOS_FREERTOS
//#define ROCKETRTOS_TEENSYTHREADS
#define ROCKETRTOS_NONE





#include <Arduino.h>
#ifdef ROCKETRTOS_MBED
#include <mbed.h>
#include <rtos.h>
#endif //ROCKETRTOS_MBED
#ifdef ROCKETRTOS_FREERTOS
#include <FreeRTOS_TEENSY4.h>
#endif //ROCKETRTOS_FREERTOS
#ifdef ROCKETRTOS_TEENSYTHREADS
#include <TeensyThreads.h>
#endif //ROCKETRTOS_TEENSYTHREADS

void startRocketRTOS();

//should be defined in another file, else state will allways be ROCKET_FREEFALL
void determineState();

typedef enum {
    ROCKET_BOOT = 0,
    ROCKET_PRE = 1,
    ROCKET_LAUNCH = 2,
    ROCKET_FREEFALL = 3,
    ROCKET_RECOVERY = 4
} RocketState_t;
extern RocketState_t rocketState;

#define THREADS_STACK_DEPTH 1024
//extern to expose them just in case
#ifdef ROCKETRTOS_MBED
extern rtos::Thread sensorAndControlThread;
extern rtos::Thread loggingThread;
extern rtos::Thread stepperThread;
extern rtos::Thread buzzerThread;
#endif //ROCKETRTOS_MBED
#ifdef ROCKETRTOS_FREERTOS
extern TaskHandle_t sensorAndControlThread;
extern TaskHandle_t loggingThread;
extern TaskHandle_t stepperThread;
extern TaskHandle_t buzzerThread;
extern TaskHandle_t stateCheckingThread;
#endif //ROCKETRTOS_FREERTOS


//All delays are in addition to the execution time, meaning the period is
//delay + execution time. The execution time was measured in STATE_TESTING
//and therefore has some over-head built in, and the real execution time 
//is slightly lower

//Execution time in STATE_TESTING: ~20 ms
#define SENSOR_AND_CONTROL_DELAY_MS 80 //100 ms -> 10 Hz

//Execution time in STATE_TESTING: ~55 ms
#define LOGGING_DELAY_MS 145  //200 ms -> 5 Hz

//Execution time in STATE_TESTING: 0 ms
#define STATE_CHECKING_DELAY_MS 100 //100 ms -> 10 Hz

//Execution time of buzz length, blocks stepper but not other tasks
//Set this delay to determine the gap between status buzz codes
#define BUZZ_DELAY_MS 2000

//Stepper Tasks
void stepper_RUN(); //should implement full stepper functionality
void stepper_CLOSE(); //should close stepper flaps
void stepper_IDLE(); //called when stepper is not needed

//Sensor and Control Tasks
void sensorAndControl_PRE(); //should gather data but not integrate or control
void sensorAndControl_LAUNCH(); //should gather data and integrate but not control
void sensorAndControl_FULL(); //should gather data and integrate and control 
void sensorAndControl_IDLE(); //called when sensors and control not needed

//Logging Tasks
void logging_RUN(); //should log data
void logging_CLOSE(); //should save the file
void logging_IDLE(); //called when logging not needed

//Buzzer Tasks
void buzz_PRE(); //Pre launch buzzes
void buzz_POST(); //Post recovery buzzes
void buzz_IDLE(); //called when buzzing not needed

#endif //ROCKET_RTOS_HH
