#include "HardwarePWM.hh"

#include <Arduino.h>
#include <nrf.h>
#include <nrf_gpio.h>
#include <nrf_pwm.h>
#include <nrf_timer.h>

void pwmSetup(){ 
    //turn the thing on
    nrf_pwm_enable(NRF_PWM0);

    //set gpio pin for D9 because that's what we've been using for the stepper motor step
    uint32_t pwmPins[NRF_PWM_CHANNEL_COUNT] = {P0_27, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED};
    nrf_pwm_pins_set(NRF_PWM0, pwmPins);

    //set clock, set count mode, and set frequency
    uint16_t resetValue = (uint16_t)(16000000U/PWM_PPS);
    nrf_pwm_configure(NRF_PWM0, NRF_PWM_CLK_16MHz, NRF_PWM_MODE_UP, resetValue)

    //set the compare register to half the upcount value to set 50% duty cycle, then set to 0%
#define sequenceSize 2
    uint16_t compareValue[sequenceSize] = {resetValue/2, resetValue+1}; //polarity?
    nrf_pwm_sequence_t seq0 = {
        .values.p_common = compareValue; //leaving the msb untouched leaves the output in the default polarity
        .length = sequenceSize; 
        //.repeats? .end_delay? probably don't need them
        };
    nrf_pwm_sequence_set(NRF_PWM0, 0, seq0);

    //trigger the conversions??????
    nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0); //might need to consider clearing events beforehand tho

    
    

    //To toggle pwm on and off I can just set a two-element sequence
    //where the second has a compare value greater than the reset value?
    //Then, using a timer, manually trigger the sequence step task to toggle?
    
    //I'd use Timer0, but I'm concerned Arduino might already be using it
    
    //TODO:
        //Start a Timer as a Counter
        //Set the compare register to the desired number of pulses
        //Connect the Timer Count task to rising edge in the gpio //look at the PPI and the GPIOTE //may require shorting the output gpio to another in input mode //actually, looks like I can force both input and output mode to be active at once
        //Connect the Timer Compare event to the PWM Stop task
        

}

void pwmPulseNPulses(unsigned long int pulses){

}
