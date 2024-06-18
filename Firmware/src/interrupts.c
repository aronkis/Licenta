#include "../include/interrupts.h"
#include "../include/functions.h"
#include "../include/serial.h"

#include <avr/wdt.h>

volatile uint8_t adcValue;
volatile uint8_t adcInt;
volatile uint8_t adcTime;
volatile uint8_t adcRead;
volatile uint8_t compFlag;
volatile uint16_t count = 0;
volatile uint16_t count1 = 0;
volatile uint16_t phaseDelay;
volatile uint8_t zeroCrossPolarity;
volatile uint16_t thirtyDegreeTime = 0;
volatile uint16_t sixtyDegreeTimes[6];
volatile uint16_t thirtyDegreeTimesave;
volatile uint16_t thirtyDegreeTime1000;
volatile uint16_t thirtyDegreeTime12;
volatile uint16_t TCNT1Save;

volatile uint8_t speedUpdated = 0;
volatile uint8_t adcFlag = 0;
volatile uint8_t speedRef = 0;
volatile uint8_t refVolt = 0;
volatile uint8_t motorFlag = 0;
volatile uint8_t ADMUXSave = 0;
volatile uint16_t speedRefMap = 0;



void TIMER0_OVF_vect(void)
    __attribute__((__signal__))
    __attribute__((__used__));
void TIMER0_OVF_vect(void)
{
    if (speedRef < 120) 
    {
        speedRef = 120;
    }
    SET_COMPx_TRIGGER_VALUE(OCR0B, speedRef);
}

//__externally_visibile__ if does not compile into the binary
void  ADC_vect(void)
    __attribute__((__signal__))
    __attribute__((__used__));
void ADC_vect(void) //ZC Detection and current measurements
{
    ADCSRA |= SET_BIT(ADIF);

    if (adcFlag == 0)
    {
        adcRead = ADCH + 5;

        if (((zeroCrossPolarity == RISING)  && (adcRead >= ZC_DETECTION_THRESHOLD)) || 
            ((zeroCrossPolarity == FALLING) && (adcRead <= ZC_DETECTION_THRESHOLD)))
        {
            adcInt = TRUE;
            sixtyDegreeTimes[nextPhase] = TCNT1;
            TCNT1 = 0;
            thirtyDegreeTime = 0;
            for (int i = 0; i < NUMBER_OF_STEPS; i++)
            {
                thirtyDegreeTime += sixtyDegreeTimes[i];
            }
            thirtyDegreeTime *= 1000;
            thirtyDegreeTime /= 12;

            adcFlag = 1;
            OCR1A = thirtyDegreeTimesave; // - 2 - 4;
            TIFR1  |= (1 << OCF1A);
            TIMSK1 |= SET_BIT(OCIE1A);
            ADCSRA &= CLEAR_BIT(ADIE);
        }
    }
    ADCSRA |= SET_BIT(ADSC);       
}

void TIMER1_COMPA_vect(void)
    __attribute__((__signal__))
    __attribute__((__used__));
void TIMER1_COMPA_vect(void)
{
    TIFR1  |= (1 << OCF1A);
        
    TCNT1Save = TCNT1;
    adcFlag = 0;
    compFlag = TRUE;
    
    DRIVE_PORT = nextStep;
    ADMUX = ADMUXTable[nextPhase];

    CHECK_ZERO_CROSS_POLARITY;

    nextPhase++;
    if (nextPhase >= 6)
    {
        nextPhase = 0;
    }
    nextStep = driveTable[nextPhase];  

    TIMSK1 &= CLEAR_BIT(OCIE1A);
    ADCSRA = (1 << ADEN) | (1 << ADIE) | ADC_PRESCALER_8;
    ADCSRA |= (1 << ADSC); 

}

// void TIMER1_COMPB_vect(void) 
//     __attribute__((__signal__))
//     __attribute__((__used__)); 
// void TIMER1_COMPB_vect(void) // Enable ZC Detection
// {
//     TIMSK1 &= CLEAR_BIT(OCIE1B);
//     ADCSRA |= SET_BIT(ADIF) | SET_BIT(ADATE); 
//     ADCSRA |= SET_BIT(ADSC); // Start a manual converion
//     CLEAR_INTERRUPT_FLAGS(TIFR1);
// }

// void TIMER1_COMPA_vect(void) 
//     __attribute__((__signal__))
//     __attribute__((__used__)); 
// void TIMER1_COMPA_vect(void) // Commutate
// {
//     CLEAR_INTERRUPT_FLAGS(TIFR1);
    
//     DRIVE_PORT = nextStep;
//     TCNT1 = 0;

//     CHECK_ZERO_CROSS_POLARITY;

//     DISABLE_INTERRUPTS(ADCSRA, ADIE);
//     OCR1B = ZC_DETECTION_HOLDOFF_TIME;
//     SET_TIMER_INTERRUPT(TIMSK1, OCIE1B);
//     //wdt_reset();
// }



// void TIMER2_COMPB_vect(void)
//     __attribute__((__signal__))
//     __attribute__((__used__));
// void TIMER2_COMPB_vect(void)
// {
//     TIFR2 = TIFR2;
//     DRIVE_PORT = driveTable[nextPhase];
//     TCNT2 = 0;
// }

// void TIMER2_COMPA_vect(void)
//     __attribute__((__signal__))
//     __attribute__((__used__));
// void TIMER2_COMPA_vect(void)
// {
//     TIFR2 = TIFR2;
//     DRIVE_PORT = pullDownTable[nextPhase];
// }

// void TIMER0_OVF_vect(void)
//     __attribute__((__signal__))
//     __attribute__((__used__));
// void TIMER0_OVF_vect(void) // Read speed reference
// {
//     if (conversionFlag == FALSE)
//     {
//         conversionFlag = TRUE;
//     }
// } 

// ISR(WDT_vect)
// {
//     CLEAR_REGISTER(DRIVE_PORT);
//     while(1);
// }