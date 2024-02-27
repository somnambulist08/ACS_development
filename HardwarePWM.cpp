#include "HardwarePWM.hh"

#include <Arduino.h>
#include <nrf.h>
#include <nrf_gpio.h>
#include <nrf_gpiote.h>
#include <nrf_ppi.h>
#include <nrf_pwm.h>
#include <nrf_timer.h>

nrf_pwm_sequence_t seq0 = {0};/*{
    //.values.p_common = compareValue, //leaving the msb untouched leaves the output in the default polarity
    //.length = SEQUENCE_LENGTH, 
    //.repeats? .end_delay? probably don't need them
}; */

void pwmSetup(){ 
    //turn the thing on
    nrf_pwm_enable(NRF_PWM0);

    //set gpio pin for D9 because that's what we've been using for the stepper motor step
    uint32_t pwmPins[NRF_PWM_CHANNEL_COUNT] = {P0_27, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED};
    nrf_pwm_pins_set(NRF_PWM0, pwmPins);

    //set clock, set count mode, and set frequency
    uint16_t resetValue = (uint16_t)(16000000U/PWM_PPS);
    nrf_pwm_configure(NRF_PWM0, NRF_PWM_CLK_16MHz, NRF_PWM_MODE_UP, resetValue);

    //set the compare register to half the upcount value to set 50% duty cycle, then set to 0%
    uint16_t compareValue[SEQUENCE_LENGTH] = {resetValue/2, resetValue+1}; //polarity?
    seq0.values.p_common = compareValue;
    seq0.length = SEQUENCE_LENGTH;
    nrf_pwm_sequence_set(NRF_PWM0, 0, &seq0);

    //trigger the conversions
    //nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0); //might need to consider clearing events beforehand tho

    
    

    //To toggle pwm on and off I can just set a two-element sequence
    //where the second has a compare value greater than the reset value.
    //Then, using a timer, manually trigger the sequence step task to toggle.
    
    //I'd use Timer0, but I'm concerned Arduino might already be using it
    

    //Configure Counter
    nrf_timer_mode_set(NRF_TIMER1, NRF_TIMER_MODE_COUNTER);
    nrf_timer_bit_width_set(NRF_TIMER1, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_cc_write(NRF_TIMER1, NRF_TIMER_CC_CHANNEL0, UINT32_MAX); //set to max for now, the step for N pulses will change this later
    nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);

    //Configure GPIO Events
    pinMode(D9, OUTPUT);//double check the pins are in the right mode
    pinMode(D5, INPUT);

    nrf_gpiote_event_enable(0); //enable event channel 0
    nrf_gpiote_event_configure(0, P1_13, NRF_GPIOTE_POLARITY_LOTOHI); //rising edge trigger
    
    //Connect GPIO Event to TIMER Task
    nrf_ppi_channel_enable(NRF_PPI_CHANNEL12);//agian, I'm not sure what Arduino is using so I'll play it safe and choose 12
    nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL12, (uint32_t)nrf_gpiote_event_addr_get(NRF_GPIOTE_EVENTS_IN_0), (uint32_t)nrf_timer_task_address_get(NRF_TIMER1, NRF_TIMER_TASK_COUNT)); //Connect the rising edge trigger to the count task

    //Connect TIMER Compare Event to PWM Stop (or PWM Step) Task
    nrf_ppi_channel_enable(NRF_PPI_CHANNEL13);
    nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL13, (uint32_t)nrf_timer_event_address_get(NRF_TIMER1, NRF_TIMER_EVENT_COMPARE0), (uint32_t)nrf_pwm_task_address_get(NRF_PWM0, NRF_PWM_TASK_NEXTSTEP));
    
    //Connect the TIMER Compare Event to TIMER Clear Task as well
    //Could technically set up a fork instead of another channel but meh
    nrf_ppi_channel_enable(NRF_PPI_CHANNEL14);
    nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL14, (uint32_t)nrf_timer_event_address_get(NRF_TIMER1, NRF_TIMER_EVENT_COMPARE0), (uint32_t)nrf_timer_task_address_get(NRF_TIMER1, NRF_TIMER_TASK_CLEAR));


}

void pwmPulseNPulses(uint32_t pulses){
    //Set the number of pulses
    nrf_timer_cc_write(NRF_TIMER1, NRF_TIMER_CC_CHANNEL0, pulses);

    //Start the pulser by restarting the sequence
    nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);

    
}
