#include "../include/serial.h"
#include "../include/functions.h"
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t nextStep = 0;
volatile uint8_t nextPhase = 0;
volatile uint8_t vbusVoltage = 0;
volatile uint8_t debugMode = 0;
volatile uint8_t adcInt = FALSE;
volatile uint16_t thirtyDegreeTimesave;

void initPorts(void)
{
	DRIVE_REG = SET_BIT(AL) | SET_BIT(BL) | SET_BIT(CL) |
				SET_BIT(AH) | SET_BIT(BH) | SET_BIT(CH);
	PORTB &= CLEAR_REGISTER(PORTB);

	DDRD = SET_BIT(PWM_PIN) | SET_BIT(LED_PIN);

	DIDR0 = SET_BIT(ADC0D) | SET_BIT(ADC1D) | SET_BIT(ADC2D) |
			SET_BIT(ADC3D) | SET_BIT(ADC4D) | SET_BIT(ADC5D);
}

void initTimers(void)
{
	// Timer0 for PWM generation; Phase correct, TOP=OCR0A;
	TCCR0A |= SET_BIT(COM0B1) | SET_BIT(WGM00);
	TCCR0B |= SET_BIT(WGM02) | SET_BIT(CS00); // no prescaling
	OCR0A = PWM_TOP_VALUE;
	CLEAR_INTERRUPT_FLAGS(TIFR0);
	TIMSK0 |= SET_BIT(TOIE0); //enable for speed reference sampling

	// Timer1 for commutation timing
	TCCR1B = SET_BIT(CS11); // Prescaler 8, 1 MHz

	// // Timer2 for current chopping. 2A Limit
	// TCCR2A = 0;
	TCCR2B |= SET_BIT(CS20); // Prescaler 8, 1 MHz
	// TIMSK2 |= SET_BIT(OCIE2A) | SET_BIT(OCIE2B);
	// OCR2A = 160; //20
	// OCR2B = 240; //40 has to be measured
}

void initADC(void)
{
	motorFlag = 0;
	speedRef = 0;
	ADMUX = ADMUX_VBUS;
	ADCSRB = 0;
	ADCSRA = SET_BIT(ADEN) | SET_BIT(ADIF) | ADC_PRESCALER_8;

	ADCSRA |= SET_BIT(ADSC); // Start a manual converion
	while (!(ADCSRA & (1 << ADIF))) {} // Wait for conversion to complete

	vbusVoltage = ADCH; // Save the current VBUS voltage (it is used for ADC threshold)
	debugMode = (vbusVoltage > 50) ? 0 : 1;

	// Do not start the ramp-up until de reference voltage is high enough
	if (!debugMode)
	{
		ADMUX = ADMUX_SPD_REF;
		while (speedRef < PWM_START_VALUE)
		{
			ADCSRA |= SET_BIT(ADSC); 
			while (!(ADCSRA & (1 << ADIF))) {} // Wait for conversion to complete
			speedRef = ADCH;
		}
	}

}

// void enableWatchdogTimer(void)
// {
// 	cli();
// 	wdt_reset();
// 	WDTCSR |= SET_BIT(WDCE) | SET_BIT(WDE);
// 	WDTCSR |= SET_BIT(WDIF) | SET_BIT(WDIE) | SET_BIT(WDE) | SET_BIT(WDP2);
// 	sei();
// }

// time * delay_multipler = time in microseconds
// 50000 * 10 = 500000 [us] = 0.5 [s]
void startupDelay(uint64_t time)
{
	CLEAR_INTERRUPT_FLAGS(TIFR2);
	while (time)
	{
		TCNT2 = UINT8_MAX - DELAY_MULTIPLIER;
		// Wait for timer to overflow.
		while (!(TIFR2 & (1 << TOV2)))
		{
		}
		CLEAR_INTERRUPT_FLAGS(TIFR2);
		time--;
	};
}

void startMotor()
{
	uint8_t count2 = 0;
	SET_COMPx_TRIGGER_VALUE(OCR0B, PWM_START_VALUE);

	nextPhase = 0;
	DRIVE_PORT = driveTable[++nextPhase];
	startupDelay(STARTUP_DELAY);

	nextStep = driveTable[nextPhase];

	for (uint8_t i = 0; i < START_UP_COMMS; i++)
	{
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
	ADCSRB = 0; 
	ADCSRA = SET_BIT(ADEN) | SET_BIT(ADIE) | SET_BIT(ADIF) | ADC_PRESCALER_8;
	ADCSRA |= SET_BIT(ADSC); // Start a manual converion
	sei();
    thirtyDegreeTimesave = 2510;
	while (thirtyDegreeTimesave != 100)
	{

		if (adcInt) 
		{
			count2++;
			adcInt = FALSE;
		}
		if (count2 == 2)
		{
			if (thirtyDegreeTimesave > 110)
			{
				thirtyDegreeTimesave -= 800;
			}
			else
			{
				thirtyDegreeTimesave = 100;
				motorFlag = 1;
			}
			count2 = 0;
		}
	}
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

	pullDownTable[0] = AL_BL;
	pullDownTable[1] = AL_CL;
	pullDownTable[2] = BL_CL;
	pullDownTable[3] = BL_AL;
	pullDownTable[4] = CL_AL;
	pullDownTable[5] = CL_BL;

	ADMUXTable[0] = ADMUX_COIL_C;
	ADMUXTable[1] = ADMUX_COIL_B;
	ADMUXTable[2] = ADMUX_COIL_A;
	ADMUXTable[3] = ADMUX_COIL_C;
	ADMUXTable[4] = ADMUX_COIL_B;
	ADMUXTable[5] = ADMUX_COIL_A;

	CurrentTable[0] = ADMUX_CURR_A;
	CurrentTable[1] = ADMUX_CURR_A;
	CurrentTable[2] = ADMUX_CURR_B;
	CurrentTable[3] = ADMUX_CURR_B;
	CurrentTable[4] = ADMUX_CURR_C;
	CurrentTable[5] = ADMUX_CURR_C;

	// For startup
	startupDelays[0] = 1000;
	startupDelays[1] = 700;
	startupDelays[2] = 500;
	startupDelays[3] = 500;
	startupDelays[4] = 300;
	startupDelays[5] = 300;
}

uint8_t readChannel(uint8_t ADMUX_SETTINGS)
{
	ADMUX = ADMUX_SETTINGS;
	START_ADC_CONVERSION;
	while (!(ADCSRA & (1 << ADIF)))
	{
	} // Wait for conversion to complete
	ADCSRA |= SET_BIT(ADIF);
	return ADCH;
}

long map(long input, long in_min, long in_max, long out_min, long out_max)
{
	return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}