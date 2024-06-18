#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_

#include <avr/interrupt.h>

#define PWM_IN_MIN 1000
#define PWM_IN_MAX 2000
#define PWM_SAMPLES 50

#define COMMUTATION_CORRECTION 50
#define COMMUTATION_TIMING_IIR_COEFF_A 1
#define COMMUTATION_TIMING_IIR_COEFF_B 3

#define FILTER_PHASE_DELAY 58 //[us] should calculate with in function of the prescaler
#define ADC_DELAY 13
#define INTERRUPT_DELAY 4
#define PROCESSING_DELAY (INTERRUPT_DELAY) //[us] time to make an ADC conversion

extern volatile uint8_t motorState;


void TIMER0_OVF_vect(void)
    __attribute__((__signal__))
    __attribute__((__used__));
void TIMER0_OVF_vect(void); // Read speed reference

void  ADC_vect(void) //ZC Detection and current measurements
    __attribute__((__signal__))
    __attribute__((__used__));

void TIMER1_COMPA_vect(void)  // Commutate
    __attribute__((__signal__))
    __attribute__((__used__)); 

void TIMER1_COMPB_vect(void)  // Enable ZC Detection
    __attribute__((__signal__))
    __attribute__((__used__)); 

void TIMER2_COMPA_vect(void)   // Turn off the active phase
    __attribute__((__signal__))
    __attribute__((__used__));

void TIMER2_COMPB_vect(void)  // Turn on the active phase
    __attribute__((__signal__))
    __attribute__((__used__));


#endif