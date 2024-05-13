#include "../include/serial.h"
#include "../include/functions.h"
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t currentHighside = 0;
volatile uint8_t nextStep  = 0;
volatile uint8_t nextPhase = 0;
volatile uint8_t motorState = 0;
volatile uint8_t zeroCrossPolarity = 0;
volatile uint8_t vbusVoltage = 0;
volatile uint8_t debugMode = 0;
volatile uint16_t motorTurnOffCounter = 0;
volatile uint16_t filteredTimeSinceCommutation = 0;

void initPorts(void)
{
    DRIVE_REG |= SET_BIT(AL) | SET_BIT(BL) | SET_BIT(CL) |
			     SET_BIT(AH) | SET_BIT(BH) | SET_BIT(CH);
	CLEAR_REGISTER(PORTB);

	DDRD |= SET_BIT(PWM_PIN);

	DIDR0 |= SET_BIT(ADC0D) | SET_BIT(ADC1D) | SET_BIT(ADC2D) |
		     SET_BIT(ADC3D) | SET_BIT(ADC4D) | SET_BIT(ADC5D); 
}

void initTimers(void)
{
	// Timer0 for PWM generation; Phase correct, TOP=OCR0A;
	TCCR0A |= SET_BIT(COM0B1) | SET_BIT(WGM00);
	TCCR0B |= SET_BIT(WGM02) | SET_BIT(CS00); // no prescaling
	OCR0A  = PWM_TOP_VALUE;
	CLEAR_INTERRUPT_FLAGS(TIFR0);
	TIMSK0 = (0 << TOIE0);

    // Timer1 for commutation timing
	TCCR1B = (1 << CS11); // Prescaler 8, 1 MHz

    // Timer2 for PWM measuring TBD
    // TCCR2A = 0;
    // TCCR2B = 0;
    // TIMSK2 |= SET_BIT(TOIE2);

    // // Enable interrupt on pin change
    // PCICR  |= SET_BIT(PCIEx);
    // PCMSK0 |= SET_BIT(PCINTx);
}

void initADC(void)
{
	ADMUX = ADC_VBUS_PIN;
	ADCSRA |= SET_BIT(ADEN) | SET_BIT(ADSC) | SET_BIT(ADIF) | ADC_PRESCALER_8;
	while (ADCSRA & (1 << ADSC)) {}
	vbusVoltage = ADCH;
	debugMode = (vbusVoltage > 1000) ? 0 : 1;	

	ADCSRA &= CLEAR_BIT(ADSC);
	ADCSRA |= SET_BIT(ADEN) | SET_BIT(ADIF) | SET_BIT(ADATE) | ADC_PRESCALER_8;
}

// TODO: Should only be used above 8k RPM
void initComparator(void)
{
#ifdef COMPARATOR_MEASURE
    //ADCSRA &= CLEAR_BIT(ADEN);
    //ADCSRB |= SET_BIT(ACME);
    ACSR    = (0 << ACBG) | SET_BIT(ACIE) | SET_BIT(ACI) | SET_BIT(ACIS1) | SET_BIT(ACIS0);
#endif
}

// void enableWatchdogTimer(void)
// {
// 	cli();
// 	wdt_reset();
// 	WDTCSR |= SET_BIT(WDCE) | SET_BIT(WDE);
// 	WDTCSR |= SET_BIT(WDIF) | SET_BIT(WDIE) | SET_BIT(WDE) | SET_BIT(WDP2);
// 	sei();
// }

void startupDelay(uint16_t time)
{
	CLEAR_INTERRUPT_FLAGS(TIFR1);
	do
	{
		TCNT1 = UINT16_MAX - DELAY_MULTIPLIER;
		// Wait for timer to overflow.
		while (!(TIFR1 & (1 << TOV1)));
		CLEAR_INTERRUPT_FLAGS(TIFR1);
		time--;
	} while (time);
}

void startMotor()
{
	uint8_t i;
	SET_COMPB_TRIGGER_VALUE(PWM_START_VALUE);

	nextPhase = 0;
	//debug_print(nextPhase, "CURRENTLY_IN_PHASE: ");
	DRIVE_PORT = driveTable[nextPhase];
	startupDelay(STARTUP_DELAY);
	// DRIVE_PORT = driveTable[++nextPhase];
	nextPhase++;
	nextStep = driveTable[nextPhase];

	for (i = 0; i < START_UP_COMMS; i++)
	{
		//debug_print(nextPhase, "CURRENTLY_IN_PHASE: ");
		DRIVE_PORT = nextStep;
		startupDelay(startupDelays[i]);

		ADMUX = ADMUXTable[nextPhase];
		CHECK_ZERO_CROSS_POLARITY;

		nextPhase++;
		if (nextPhase >= 6)
		{
			nextPhase = 0;
		}
		nextStep = driveTable[nextPhase];
	}

	// Soft start done.
	TCNT1 = 0;
	filteredTimeSinceCommutation = startupDelays[START_UP_COMMS - 1] * (DELAY_MULTIPLIER / 2);
	OCR1B = ZC_DETECTION_HOLDOFF_TIME;
	SET_TIMER1_HOLDOFF_INT;
}

void generateTables(void)
{
	// Mosfet sequence
	driveTable[0] = AH_BL;
	driveTable[1] = AH_CL;
	driveTable[2] = BH_CL;
	driveTable[3] = BH_AL;
	driveTable[4] = CH_AL;
	driveTable[5] = CH_BL; 

	ADMUXTable[0] = ADC_COIL_C_PIN;
	ADMUXTable[1] = ADC_COIL_B_PIN;
	ADMUXTable[2] = ADC_COIL_A_PIN;
	ADMUXTable[3] = ADC_COIL_C_PIN;
	ADMUXTable[4] = ADC_COIL_B_PIN;
	ADMUXTable[5] = ADC_COIL_A_PIN;

	// For startup
	startupDelays[0] = 200;
  	startupDelays[1] = 150;
  	startupDelays[2] = 100;
  	startupDelays[3] = 80;
  	startupDelays[4] = 70;
  	startupDelays[5] = 65;
  	startupDelays[6] = 60;
  	startupDelays[7] = 55;
}

void runMotor(void)
{
	if(motorState)
	{
		motorTurnOffCounter = 0;
		startMotor();
		while (motorState)
		{
			//TODO: motor turn off steps
		}
	}
}

uint8_t readChannel(uint8_t adcChannel)
{
	ADMUX = adcChannel;
	START_ADC_CONVERSION;
	while (ADCSRA & (1 << ADSC)) {}
	return ADCH;
}