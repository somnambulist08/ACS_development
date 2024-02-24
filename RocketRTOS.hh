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
void loop2();
void loop3();

#endif //ROCKET_RTOS_HH
