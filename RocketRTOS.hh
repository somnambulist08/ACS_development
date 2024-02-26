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


/*************************************************************************
 * Turns out I have even more reason to hate Arduino because they have
 * suppressed the mbed library and made their own shitty verion. YAY!!!!
**************************************************************************/
/*
#include <mbed.h>
#include <rtos.h>

extern Thread t;

void threadHelloWorld(void);
*/

#include <Scheduler.h>

void startRocketRTOS();

//#define SERIAL_STEPPER_TEST
#ifdef SERIAL_STEPPER_TEST

#include "SerialSpoofStepper.hh"
//#include "InternalSensors.hh"
#include "SDSpoofer.hh"
#include "Control.hh"

#define STEPS_PER_PREEMPT 10
void stepperTask();
extern SerialSpoofStepper stepper;

#define CONTROL_PERIOD_MS 100
void sensorAndControlTask();
//extern sixteenAMG sensor;
extern unsigned long oldMicros;
extern float velocity;
extern float oldAccel;

#define LOG_PERIOD_MS 1000
void logTask();
extern SDSpoofer dummySD;




#endif //SERIAL_STEPPER_TEST


//#define LOOP_CHECKIN_TEST
#ifdef LOOP_CHECKIN_TEST
void loop2();
void loop3();
#endif //LOOP_CHECKIN_TEST

//#define DUMMY_FUNCTIONS
#ifdef DUMMY_FUNCTIONS

#define SENSOR_PERIOD_MS 100
void getSensorsAndDoControlTask();

#define FILE_PERIOD_MS 2500
void logToFileTask();


/* This would be an implementation with 
 * moving the stepper as a periodic task.
 * Ideally, the stepper should also have an
 * aperiodic component (or be purely aperiodic)
 * that takes over during any slack between other
 * tasks. The processor should never be idle
 * and at this time I'm unsure how to 
 * over-write the idletask.
 * 
 * holup
 * 
 * idea
 * 
 * what if I treat the main loop as the idle task?
 * set it up with no delay but have it yield after
 * every iteration.
 * holy shit
 * that might work
 * 
 * 
 * 
 * 
 * ALSO we can save clock cycles by having this
 * task control a pwm hardware timer, but I have
 * not figured out how to access hardware timers
 * and identified what they are actually capable 
 * of just yet. 
 *
*/
#define STEPPER_PERIOD_MS 10
void moveStepperTask();


#endif //DUMMY_FUNCTIONS




#endif //ROCKET_RTOS_HH
