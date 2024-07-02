#include "../include/interrupts.h"
#include "../include/functions.h"
#include "../include/serial.h"

// external variables
volatile enum ADC_STATE commutationState = ADC_BEMF_READ;
volatile uint8_t backEMFFound;
volatile uint8_t zeroCrossPolarity;
volatile uint8_t speedReference = 0;
volatile uint8_t speedReferenceSave = 0;
volatile uint8_t motorStarted = FALSE;
volatile uint16_t thirtyDegreeTime = 0;
volatile uint16_t motorStartupDelay;
volatile uint16_t motorStopCounter;

// static variables
volatile uint8_t backEMFValue;
volatile uint8_t speedUpdated = 0;
volatile uint16_t sixtyDegreeTimes[6];

void TIMER0_OVF_vect(void) // PWM value update
{
    if (speedUpdated)
    {
        OCR0B = speedReference;
        speedUpdated = FALSE;
    }
}

void ADC_vect(void) // ZC detection and speed reference measurement
{
    ADCSRA |= SET_BIT(ADIF);

    switch (commutationState)
    {
        case ADC_BEMF_READ: // checking if a zero-cross occured
            backEMFValue = ADCH;

            if (((zeroCrossPolarity == RISING)  && (backEMFValue > ZC_DETECTION_THRESHOLD)) || 
                ((zeroCrossPolarity == FALLING) && (backEMFValue < ZC_DETECTION_THRESHOLD)))
            {
                sixtyDegreeTimes[nextPhase] = TCNT1;
                TCNT1 = 0;
                thirtyDegreeTime = getThirtyDegreeTime();
                
                if (motorStarted)
                {
                    OCR1A = thirtyDegreeTime;
                }
                else
                {
                    backEMFFound = TRUE;
                    OCR1A = motorStartupDelay;
                }

                commutationState = ADC_SPEED_REFERENCE_READ;
                ADMUX = ADMUX_SPD_REF;
                TIFR1  |= SET_BIT(OCF1A);
                TIMSK1 |= SET_BIT(OCIE1A);
            }
        break;
        case ADC_SPEED_REFERENCE_READ: // reading the speed reference
            speedReference = ADCH;
            speedReferenceSave = speedReference;

            if (speedReference < PWM_MIN_VALUE)
            {
                speedReference = PWM_MIN_VALUE;
            }
            if (speedReference > PWM_MAX_VALUE)
            {
                speedReference = PWM_MAX_VALUE;
            }

            speedUpdated = TRUE;
            ADCSRA &= CLEAR_BIT(ADIE);
        break;
    }

    ADCSRA |= SET_BIT(ADSC);       
}

void TIMER1_COMPA_vect(void) // Commutation
{
    TIFR1 |= SET_BIT(OCF1A);
        
    commutationState = ADC_BEMF_READ;
    
    DRIVE_PORT = nextStep;
    ADMUX = ADMUXTable[nextPhase];

    zeroCrossPolarity = checkForZeroCrossPolarity();

    nextPhase++;
    if (nextPhase >= 6)
    {
        nextPhase = 0;
    }
    nextStep = driveTable[nextPhase];  

    TIMSK1 &= CLEAR_BIT(OCIE1A);
    ADCSRA |= SET_BIT(ADIE);    
    ADCSRA |= SET_BIT(ADSC); 
}

uint16_t getThirtyDegreeTime(void)
{
    uint16_t temp = 0;
    for (int i = 0; i < NUMBER_OF_STEPS; i++)
    {
        temp += sixtyDegreeTimes[i];
    }
    return temp / 12;

}