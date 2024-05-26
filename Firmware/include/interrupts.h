#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_

#include <avr/interrupt.h>

#define PWM_IN_MIN 1000
#define PWM_IN_MAX 2000
#define PWM_SAMPLES 50

#define COMMUTATION_CORRECTION 50
#define COMMUTATION_TIMING_IIR_COEFF_A 1
#define COMMUTATION_TIMING_IIR_COEFF_B 3

extern volatile uint8_t motorState;

//ISR(PCINTx_vect);       // Measure PWM
//ISR(TIMER2_OVF_vect);   // PWM time measuring
ISR(TIMER0_OVF_vect);   // Current measure 
ISR(TIMER1_COMPA_vect); // Commutate
ISR(TIMER1_COMPB_vect); // ZC detection enable
ISR(ADC_VECT);          // Zero crossing


void  ADC_vect(void) //ZC Detection and current measurements
    __attribute__((__signal__))
    __attribute__((__used__));

void TIMER1_COMPA_vect(void)  // Commutate
    __attribute__((__signal__))
    __attribute__((__used__)); 

void TIMER1_COMPB_vect(void)  // Enable ZC Detection
    __attribute__((__signal__))
    __attribute__((__used__)); 

#endif