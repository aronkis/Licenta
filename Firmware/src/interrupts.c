#include "../include/interrupts.h"
#include "../include/functions.h"
#include "../include/serial.h"

#include <avr/wdt.h>

volatile uint8_t zcFlag = TRUE;
volatile uint8_t speedRef = 0;
volatile uint8_t shuntVoltageCoilA = 0;
volatile uint8_t shuntVoltageCoilB = 0;
volatile uint8_t shuntVoltageCoilC = 0;
volatile uint8_t speedUpdated = FALSE;
volatile uint8_t currentUpdated = FALSE;
volatile uint8_t adcValue;

//__externally_visibile__ if does not compile into the binary
void  ADC_vect(void)
    __attribute__((__signal__))
    __attribute__((__used__));
void ADC_vect(void) //ZC Detection and current measurements
{

    ADCSRA &= CLEAR_BIT(ADATE);
    ADCSRA |= SET_BIT(ADIF);

    adcValue = ADCH;

    if (zcFlag)
    {
        if (((zeroCrossPolarity == RISING) && (adcValue > ZC_DETECTION_THRESHOLD)) || 
            ((zeroCrossPolarity == FALLING) && (adcValue < ZC_DETECTION_THRESHOLD)))
        {
            uint16_t timeSinceCommutation = TCNT1;
            TCNT1 = COMMUTATION_CORRECTION;
            filteredTimeSinceCommutation = ((COMMUTATION_TIMING_IIR_COEFF_A * timeSinceCommutation +
                                            COMMUTATION_TIMING_IIR_COEFF_B * filteredTimeSinceCommutation) /
                                            (COMMUTATION_TIMING_IIR_COEFF_A + COMMUTATION_TIMING_IIR_COEFF_B)); // time related
            OCR1A = filteredTimeSinceCommutation; 

            setCommutationTimer();
            clearTimer1InterruptFlags();

            speedRef = readChannel(ADC_SPD_REF_PIN);
            //vbusVoltage = readChannel(ADC_VBUS_PIN);
            
            speedUpdated = TRUE; 
            zcFlag = FALSE;
        }
    }
    else
    {
        uint8_t tempADMUX = ADMUX;

        // read the current only on the current HighSide resistor
        shuntVoltageCoilA = readChannel(CurrentTable[nextPhase]);

        currentUpdated = TRUE;
        changeChannel(tempADMUX);
    }

    ADCSRA |= SET_BIT(ADATE);
}

void TIMER1_COMPA_vect(void) 
    __attribute__((__signal__))
    __attribute__((__used__)); 
void TIMER1_COMPA_vect(void) // Commutate
{
    clearTimer1InterruptFlags();
    
    DRIVE_PORT = nextStep;
    TCNT1 = 0;

    checkZeroCrossPolarity();

    disableADCInterrupts();
    OCR1B = zcDetectionHoldoffTime();
    setADCHoldoffInterrupt();
    //wdt_reset();
}

void TIMER1_COMPB_vect(void) 
    __attribute__((__signal__))
    __attribute__((__used__)); 
void TIMER1_COMPB_vect(void) // Enable ZC Detection
{
    clearTimer1InterruptFlags();
    disableTimer1Interrupts();
    zcFlag = TRUE;

    changeChannel(ADMUXTable[nextPhase]);
    startADCConversion();

    nextPhase++;
    if (nextPhase >= 6)
    {
        nextPhase = 0;
    }
    nextStep = driveTable[nextPhase];
}

// ISR(WDT_vect)
// {
//     CLEAR_REGISTER(DRIVE_PORT);
//     while(1);
// }

//volatile uint8_t timerValue = 0;
//volatile uint32_t PWMInput = 0;
//volatile uint8_t lastPWMPinState = 0;
//volatile uint8_t lastTimerState = 0;
//volatile uint8_t PWMAverageCount = 0;
//volatile uint8_t timerOverflowCounter = 0;

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