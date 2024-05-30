#include "../include/interrupts.h"
#include "../include/functions.h"
#include "../include/serial.h"

#include <avr/wdt.h>

volatile uint8_t adcValue;
volatile uint8_t adcFlag;
volatile uint8_t adcInt;
volatile uint8_t adcTime;
volatile uint8_t adcRead;
volatile uint8_t compFlag;
volatile uint16_t count = 0;;
volatile uint16_t phaseDelay;
volatile uint8_t zeroCrossPolarity;
volatile uint16_t thirtyDegreeTime = 0;
volatile uint16_t sixtyDegreeTimes[6];
volatile uint16_t thirtyDegreeTimesave;
volatile uint16_t thirtyDegreeTime1000;
volatile uint16_t thirtyDegreeTime12;
volatile uint16_t TCNT1Save ;


//__externally_visibile__ if does not compile into the binary
void  ADC_vect(void)
    __attribute__((__signal__))
    __attribute__((__used__));
void ADC_vect(void) //ZC Detection and current measurements
{
    // adcValue = ADCH;

    // if (((zeroCrossPolarity == RISING) && (adcValue > ZC_DETECTION_THRESHOLD)) || 
    //     ((zeroCrossPolarity == FALLING) && (adcValue < ZC_DETECTION_THRESHOLD)))
    // {
    //     GREEN_LED;
    // 	SET_COMPx_TRIGGER_VALUE(OCR0B, PWM_START_VALUE);
    //     DRIVE_PORT = nextStep;
    //     changeChannel(ADMUXTable[nextPhase]);
    //     nextPhase++;
    //     if (nextPhase >= 6)
    //     {
    //         nextPhase = 0;
    //     }
    //     nextStep = driveTable[nextPhase];
    //     ADCSRA &= CLEAR_BIT(ADATE);
    //     TIMSK1 = SET_BIT(OCIE1B);
    //     OCR1B = 30;
    //     TCNT1 = 0;
    // }
    // else
    // {
    //     RED_LED;
    // }
    adcRead = ADCH;
    adcInt = TRUE;
    if (((zeroCrossPolarity == RISING)  && (adcRead >= ZC_DETECTION_THRESHOLD)) || 
        ((zeroCrossPolarity == FALLING) && (adcRead <= ZC_DETECTION_THRESHOLD)))
    {
        ADCSRA &= CLEAR_BIT(ADIE);
       
        sixtyDegreeTimes[nextPhase] = TCNT1Save/1000;
        TCNT1 = 0;
        //adcTime = TCNT1;
        adcFlag = TRUE;
        phaseDelay = FILTER_PHASE_DELAY + PROCESSING_DELAY;
        thirtyDegreeTime = 0;
        for (int i = 0; i < NUMBER_OF_STEPS; i++)
        {
            thirtyDegreeTime += sixtyDegreeTimes[i];
        }
        thirtyDegreeTimesave = thirtyDegreeTime;
        thirtyDegreeTime *= 1000;
        thirtyDegreeTime /= 12;
        // thirtyDegreeTime = 2500;

        if (thirtyDegreeTime < phaseDelay)
        {
            thirtyDegreeTime = phaseDelay;
        }
    
        // changeChannel(ADMUX_VBUS);
        // ADCSRA |= SET_BIT(ADSC); // Start a manual converion
	    // while (!(ADCSRA & (1 << ADIF))) {} // Wait for conversion to complete
	    // vbusVoltage = ADCH - 20; // Save the current VBUS voltage (it is used for ADC threshold)

        OCR1A = thirtyDegreeTime - phaseDelay;
        TIMSK1 |= SET_BIT(OCIE1A);
    }
    ADCSRA |= SET_BIT(ADIF);
}

void TIMER1_COMPA_vect(void)
    __attribute__((__signal__))
    __attribute__((__used__));
void TIMER1_COMPA_vect(void)
{
    TCNT1Save = TCNT1;
    TIFR1 = TIFR1;

    DRIVE_PORT = nextStep;
    changeChannel(ADMUXTable[nextPhase]);

    CHECK_ZERO_CROSS_POLARITY;

    nextPhase++;
    if (nextPhase >= 6)
    {
        nextPhase = 0;
    }
    nextStep = driveTable[nextPhase];  
    TIMSK1 &= CLEAR_BIT(OCIE1A);
    ADCSRA |= SET_BIT(ADIE);
    ADCSRA |= SET_BIT(ADSC);
    compFlag = TRUE;
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



void TIMER2_COMPB_vect(void)
    __attribute__((__signal__))
    __attribute__((__used__));
void TIMER2_COMPB_vect(void)
{
    DRIVE_PORT = driveTable[nextPhase];
    TCNT2 = 0;
}

void TIMER2_COMPA_vect(void)
    __attribute__((__signal__))
    __attribute__((__used__));
void TIMER2_COMPA_vect(void)
{
    DRIVE_PORT = pullDownTable[nextPhase];
}

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