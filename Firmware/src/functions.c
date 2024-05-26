#include "../include/serial.h"
#include "../include/functions.h"
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t currentHighside = 0;
volatile uint8_t nextStep = 0;
volatile uint8_t nextPhase = 0;
volatile uint8_t motorState = 0;
volatile uint8_t zeroCrossPolarity = 0;
volatile uint8_t vbusVoltage = 0;
volatile uint8_t debugMode = 0;
volatile uint16_t motorTurnOffCounter = 0;
volatile uint16_t filteredTimeSinceCommutation = 0;
volatile uint16_t current = 0;
volatile uint16_t count = 0;

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
	TIMSK0 = (0 << TOIE0);

	// Timer1 for commutation timing
	TCCR1B = (1 << CS11); // Prescaler 8, 1 MHz

	// Set up timer 2 for 10 and 50 uS interrupts
	// pull high on the first 10, then pull low on the next 50 then reset
}

void initADC(void)
{
	ADMUX = ADMUX_VBUS;
	ADCSRB = 0;
	ADCSRA = SET_BIT(ADEN) | ADC_PRESCALER_8; //| SET_BIT(ADIE) | SET_BIT(ADIF) |SET_BIT(ADATE)

	ADCSRA |= SET_BIT(ADSC); // Start a manual converion
	while (!(ADCSRA & (1 << ADIF))) {} // Wait for conversion to complete

	vbusVoltage = ADCH; // Save the current VBUS voltage (it is used for ADC threshold)
	debugMode = (vbusVoltage > 100) ? 0 : 1;
}

// TODO: Should only be used above 8k RPM
void initComparator(void)
{
#ifdef COMPARATOR_MEASURE
	// ADCSRA &= CLEAR_BIT(ADEN);
	// ADCSRB |= SET_BIT(ACME);
	ACSR = (0 << ACBG) | SET_BIT(ACIE) | SET_BIT(ACI) | SET_BIT(ACIS1) | SET_BIT(ACIS0);
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

void startupDelay(uint32_t time)
{
	CLEAR_INTERRUPT_FLAGS(TIFR1);
	while (time)
	{
		TCNT1 = UINT16_MAX - DELAY_MULTIPLIER;
		// Wait for timer to overflow.
		while (!(TIFR1 & (1 << TOV1)))
		{
		}
		CLEAR_INTERRUPT_FLAGS(TIFR1);
		time--;
	};
}

void startMotor()
{
	uint8_t i;
	uint8_t shuntVoltage = 0;
	uint16_t count = 0;

	SET_COMPx_TRIGGER_VALUE(OCR0B, PWM_START_VALUE);

	nextPhase = 0;
	DRIVE_PORT = driveTable[nextPhase];
	startupDelay(STARTUP_DELAY);
	// DRIVE_PORT = driveTable[++nextPhase];
	nextPhase++;
	nextStep = driveTable[nextPhase];

	for (i = 0; i < START_UP_COMMS; i++)
	{
		// debug_print(nextPhase, "phase = ");
		DRIVE_PORT = nextStep;
		startupDelay(startupDelays[i]);

		changeChannel(ADMUXTable[nextPhase]);

		CHECK_ZERO_CROSS_POLARITY;

		// shuntVoltage = 1 + readChannel(CurrentTable[nextPhase]);
		// current = (shuntVoltage * ADC_REFERENCE / ADC_RESOLUTION * 1000) / SHUNT_RESISTANCE; // I = U/R [mA]

		// nextPhase = (++nextPhase == NUMBER_OF_STEPS) ? 0 : nextPhase++;
		nextPhase++;
		if (nextPhase >= NUMBER_OF_STEPS)
		{
			nextPhase = 0;
		}
		nextStep = driveTable[nextPhase];

		if (i == 11)
		{
			i = 9;
		}
		count++;
		if (count == 1000)
		{
			break;
		}
	}

	// Soft start done.
	TCNT1 = 0;
	filteredTimeSinceCommutation = startupDelays[START_UP_COMMS - 1] * (DELAY_MULTIPLIER / 2);
	OCR1B = ZC_DETECTION_HOLDOFF_TIME;
	SET_TIMER_INTERRUPT(TIMSK1, OCIE1B);
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
	startupDelays[0] = 20;
	startupDelays[1] = 15;
	startupDelays[2] = 10;
	startupDelays[3] = 8;
	startupDelays[4] = 8;
	startupDelays[5] = 7;
	startupDelays[6] = 7;
	startupDelays[7] = 6;
	startupDelays[8] = 6;
	startupDelays[9] = 3;
	startupDelays[10] = 3;
	startupDelays[11] = 3;
}

void runMotor(void)
{
	if (motorState)
	{
		motorTurnOffCounter = 0;
		startMotor();
		while (motorState)
		{
			// TODO: motor turn off steps
		}
	}
}

// TODO: PAGE251
void changeChannel(uint8_t ADMUX_SETTINGS)
{
	ADCSRA &= CLEAR_BIT(ADATE);
	ADMUX = ADMUX_SETTINGS;
	ADCSRA |= SET_BIT(ADATE);
}

uint8_t readChannel(uint8_t ADMUX_SETTINGS)
{
	changeChannel(ADMUX_SETTINGS);
	START_ADC_CONVERSION;
	while (!(ADCSRA & (1 << ADIF))) {} // Wait for conversion to complete
	ADCSRA |= SET_BIT(ADIF);
	return ADCH;
}

uint8_t map(uint16_t input, uint16_t in_min, uint16_t in_max, uint8_t out_min, uint8_t out_max)
{
	return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
