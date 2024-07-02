#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_

#include <avr/interrupt.h>

void __attribute__((__signal__)) 
     __attribute__((__used__))
TIMER0_OVF_vect(void); // PWM value update

void __attribute__((__signal__))
     __attribute__((__used__))
ADC_vect(void); // ZC detection and speed reference measurement

void __attribute__((__signal__))
     __attribute__((__used__))
TIMER1_COMPA_vect(void);  // Commutate

uint16_t getThirtyDegreeTime(void);

#endif //_INTERRUPTS_H_