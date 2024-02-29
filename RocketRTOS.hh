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

#define THREADS_STACK_DEPTH 1024
//extern to expose them just in case
extern rtos::Thread sensorAndControlThread;
extern rtos::Thread loggingThread;

#define SENSOR_AND_CONTROL_DELAY_MS 100
#define LOGGING_DELAY_MS 1000

void stepperTask();
void sensorAndControlTask();
void loggingTask();

#endif //ROCKET_RTOS_HH
