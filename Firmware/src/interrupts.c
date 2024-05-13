#include "../include/interrupts.h"
#include "../include/functions.h"
#include "../include/serial.h"

#include <avr/wdt.h>

volatile uint8_t lastPWMPinState = 0;
volatile uint8_t lastTimerState = 0;
volatile uint8_t speedRef = 0;
volatile uint8_t shuntVoltageCoilA = 0;
volatile uint8_t shuntVoltageCoilB = 0;
volatile uint8_t shuntVoltageCoilC = 0;
volatile uint8_t timerOverflowCounter = 0;
volatile uint8_t PWMAverageCount = 0;
volatile uint8_t speedUpdated = FALSE;
volatile uint8_t currentUpdated = FALSE;
volatile uint8_t timerValue = 0;
volatile uint32_t PWMInput = 0;


// ISR(PCINTx_vect)
// {
//     if (!lastPWMPinState)
//     {
//         lastPWMPinState = 1;
//         timerOverflowCounter = 0;
//         TCCR2B |= SET_BIT(CS20);
//         TCNT2   = 0;
//     }
//     else if (lastPWMPinState)
//     {
//         PWMInput += (TCNT2 + ((timerOverflowCounter * UINT8_MAX) >> 4));
//         PWMAverageCount++;
//         if (PWMAverageCount == PWM_SAMPLES)
//         {
//             PWMInput /= PWM_SAMPLES;
//             PWMInput = constrain(PWMInput, PWM_IN_MIN, PWM_IN_MAX);
//             motorState = (PWMInput > PWM_IN_MIN + 115) ? 1 : 0;
//             timerValue = map(PWMInput, PWM_IN_MIN, PWM_IN_MAX, \
//                                        PWM_MIN_VALUE, PWM_MAX_VALUE);
//             SET_TIMER(timerValue);
//             PWMAverageCount = 0;
//             PWMInput = 0;
//         }
//         TCCR2B = 0;
//         lastPWMPinState = 0;
//     }
// }

// ISR(TIMER2_OVF_vect)
// {
//     CLEAR_INTERRUPT_FLAGS(TIFR2);
//     timerOverflowCounter++;
// }

ISR (TIMER0_OVF_vect) // ZC detection
{
    uint8_t adcValue;

    ADCSRA &= CLEAR_BIT(ADATE);

    while (!(ADCSRA & (1 << ADIF)));

    adcValue = ADCH;

    if (((zeroCrossPolarity == RISING) && (adcValue > ZC_DETECTION_THRESHOLD)) || 
        ((zeroCrossPolarity == FALLING) && (adcValue < ZC_DETECTION_THRESHOLD)))
    {
        uint16_t timeSinceCommutation = TCNT1;
        TCNT1 = COMMUTATION_CORRECTION;
        filteredTimeSinceCommutation = ((COMMUTATION_TIMING_IIR_COEFF_A * timeSinceCommutation +
                                        COMMUTATION_TIMING_IIR_COEFF_B * filteredTimeSinceCommutation) /
                                        (COMMUTATION_TIMING_IIR_COEFF_A + COMMUTATION_TIMING_IIR_COEFF_B)); // time related
        OCR1A = filteredTimeSinceCommutation; 

        SET_TIMER1_COMMUTATE_INT;
        CLEAR_INTERRUPT_FLAGS(TIFR1);

      	WAIT_FOR_ADC_CONVERSION;

        speedRef = readChannel(ADC_SPD_REF_PIN);
        vbusVoltage = readChannel(ADC_VBUS_PIN);
        
        speedUpdated = TRUE; 
    }
    else
    {
        uint8_t tempADMUX = ADMUX;
    
       	WAIT_FOR_ADC_CONVERSION;

        shuntVoltageCoilA = readChannel(ADC_CURR_A_PIN);
        shuntVoltageCoilB = readChannel(ADC_CURR_B_PIN);
        shuntVoltageCoilC = readChannel(ADC_CURR_C_PIN);

        currentUpdated = TRUE;
        
        ADMUX = tempADMUX;
    }

    ADCSRA |= SET_BIT(ADATE);
}

ISR(TIMER1_COMPA_vect) // Commutate
{
    DRIVE_PORT = nextStep;
    TCNT1 = 0;

    CHECK_ZERO_CROSS_POLARITY;

    DISABLE_ALL_TIMER1_INTS;
    CLEAR_INTERRUPT_FLAGS(TIFR1);
    OCR1B = ZC_DETECTION_HOLDOFF_TIME;
    SET_TIMER1_HOLDOFF_INT;

    //wdt_reset();
}

ISR(TIMER1_COMPB_vect) // Enable ZC Detection
{
    CLEAR_INTERRUPT_FLAGS(TIFR0);
    CLEAR_INTERRUPT_FLAGS(TIFR1);
    DISABLE_ALL_TIMER1_INTS;

    ADMUX = ADMUXTable[nextPhase];

    while (!(ADCSRA & (1 << ADIF)));

    nextPhase++;
    if (nextPhase >= 6)
    {
        nextPhase = 0;
    }
    nextStep = driveTable[nextPhase];
}

// TODO: ADC_VECT and below
// ISR(WDT_vect)
// {
//     CLEAR_REGISTER(DRIVE_PORT);
//     while(1);
// }