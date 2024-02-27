/******************************************************************************
 * HardwarePWM.hh
 * 
 * The direct hardware access to PWM and TIMER peripherals necissary to make
 * the stepper run 100% independently of the CPU
 * 
 * NOTE:
 * The pins are connected as follows
 * D0 = P1_3 //used for TX
 * D1 = P1_10 //used for RX
 * D2 = P1_11
 * D3 = P1_12
 * D4 = P1_15
 * D5 = P1_13
 * D6 = P1_14
 * D7 = P0_23
 * D8 = P0_21
 * D9 = P0_27
 * D10 = P1_2
 * D11 = P1_1 //used for MOSI
 * D12 = P1_8 //used for MISO
 * D13 = P0_13 //used for SCK and the LED
 * 
 * 02/26/24 - Created File
******************************************************************************/
#ifndef HARDWARE_PWM_HH
#define HARDWARE_PWM_HH

#include <Arduino.h>
//750RPM -> 2500 Steps/s -> 20000 Microsteps/s
#define PWM_PPS 20000U
#define SEQUENCE_LENGTH 2U

extern nrf_pwm_sequence_t seq0;
void pwmSetup();
void pwmPulseNPulses(uint32_t pulses);



#endif //HARDWARE_PWM_HH
