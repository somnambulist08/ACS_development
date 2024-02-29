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

#include <Arduino.h>
#include <mbed.h>
#include <rtos.h>

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
extern rtos::Thread sensorAndControlThread;
extern rtos::Thread loggingThread;
extern rtos::Thread stepperThread;

#define SENSOR_AND_CONTROL_DELAY_MS 100
#define LOGGING_DELAY_MS 1000
#define STATE_CHECKING_DELAY_MS 100

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

#endif //ROCKET_RTOS_HH
