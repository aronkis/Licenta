#include "../include/serial.h"
#include "../include/functions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint8_t nextStep = 0;
volatile uint8_t nextPhase = 0;
volatile uint8_t zcThresholdVoltage = 0;
volatile uint8_t backEMFFound = FALSE;
volatile uint16_t motorStartupDelay;
enum PROGRAM_STATE programState = STARTUP;
uint8_t debugMode = 0;

void initPorts(void)
{
	DRIVE_REG = SET_BIT(AL) | SET_BIT(BL) | SET_BIT(CL) |
				      SET_BIT(AH) | SET_BIT(BH) | SET_BIT(CH);
	CLEAR_REGISTER(PORTB);

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

	// TCCR2A = 0;
	TCCR2B |= SET_BIT(CS20); // Prescaler 8, 1 MHz
}

void initADC(void)
{
	motorStarted = FALSE;
	speedReference = 0;
	ADMUX = ADMUX_VBUS;
	ADCSRB = 0;
	ADCSRA = SET_BIT(ADEN) | SET_BIT(ADIF) | ADC_PRESCALER_8;

	ADCSRA |= SET_BIT(ADSC); // Start a manual converion
	while (!(ADCSRA & SET_BIT(ADIF))) {} // Wait for conversion to complete

	zcThresholdVoltage = ADCH; // Save the current VBUS voltage (it is used for ZC threshold)
	debugMode = (zcThresholdVoltage > 50) ? 0 : 1;

	if (debugMode)
	{
		uartSendString("Debug Mode.");
		while(TRUE)
		{
			GREEN_LED;
			_delay_ms(500);
			RED_LED;
			_delay_ms(500);
		}
	}

	// Do not start the ramp-up until the speed reference is high enough
	if (!debugMode)
	{
		ADMUX = ADMUX_SPD_REF;
		while (speedReference < PWM_START_VALUE)
		{
			ADCSRA |= SET_BIT(ADSC); 
			while (!(ADCSRA & SET_BIT(ADIF))) {} // Wait for conversion to complete
			speedReference = ADCH;
		}
	}
}

// time * delay_multipler = time in microseconds
// 50000 * 10 = 500000 [us] = 0.5 [s]
void startupDelay(uint64_t time)
{
	CLEAR_INTERRUPT_FLAGS(TIFR2);
	while (time)
	{
		TCNT2 = UINT8_MAX - DELAY_MULTIPLIER;
		// Wait for timer to overflow.
		while (!(TIFR2 & SET_BIT(TOV2))) {}
		CLEAR_INTERRUPT_FLAGS(TIFR2);
		time--;
	};
}

void startMotor()
{
	GREEN_LED;

	uint8_t startupCommutationCounter = 0;
	OCR0B = PWM_START_VALUE;

	nextPhase = 0;
	DRIVE_PORT = driveTable[++nextPhase];
	startupDelay(STARTUP_DELAY);

	nextStep = driveTable[nextPhase];

	for (uint8_t i = 0; i < START_UP_COMMS; i++)
	{
		DRIVE_PORT = nextStep;
		
		startupDelay(startupDelays[i]);
		ADMUX = ADMUXTable[nextPhase];
		zeroCrossPolarity = checkForZeroCrossPolarity();

		nextPhase++;
        
		if (nextPhase >= 6)
        {
            nextPhase = 0;
        }
        nextStep = driveTable[nextPhase];
	}

	ADCSRB  = 0; 
	ADCSRA  = SET_BIT(ADEN) | SET_BIT(ADIE) | SET_BIT(ADIF) | ADC_PRESCALER_8;
<<<<<<< HEAD
	ADCSRA |= SET_BIT(ADSC); // Start a manual converion
=======
	ADCSRA |= SET_BIT(ADSC);
>>>>>>> f557f9daaca3ca2f3fbe2a281541437e801d7cf5

  motorStartupDelay = 2510;
	while (motorStartupDelay != 100)
	{
		if (backEMFFound) 
		{
			startupCommutationCounter++;
			backEMFFound = FALSE;
		}
		if (startupCommutationCounter == 2)
		{
			if (motorStartupDelay > 110)
			{
				motorStartupDelay -= 800;
			}
			else
			{
				motorStartupDelay = 100;
				motorStarted = TRUE;
				programState = RUNNING;
			}
			startupCommutationCounter = 0;
		}
	}
}

void stopMotor(void)
{
	RED_LED;
	motorStarted = FALSE;
	commutationState = ADC_BEMF_READ;
	speedReference = 0;
	motorStopCounter = 0;
	CLEAR_REGISTER(ADCSRA);
	CLEAR_REGISTER(ADMUX);
	CLEAR_REGISTER(DRIVE_PORT);
	_delay_ms(50);
}

void checkForMotorStop(void)
{
	if (speedReferenceSave < 30)
	{
		RED_LED;
		motorStopCounter++;
	}
	else
	{
		GREEN_LED;
		if (motorStopCounter > 0)
		{
			motorStopCounter--;
		}
	}
	if (motorStopCounter >= 25)
	{
		programState = RESTART;
	}
}

void checkForStartMotor(void)
{
	_delay_ms(100);

	ADMUX = ADMUX_SPD_REF;
	ADCSRA = SET_BIT(ADEN) | SET_BIT(ADIF) | ADC_PRESCALER_8;

	while (speedReference < PWM_START_VALUE)
	{
		ADCSRA |= SET_BIT(ADSC); 
		while ((ADCSRA & SET_BIT(ADSC))) {}
		ADCSRA |= SET_BIT(ADIF);

		speedReference = ADCH;
		speedReferenceSave = speedReference;

		_delay_us(50);
	}

	motorStopCounter = 0;
	programState = STARTUP;
}

uint8_t checkForZeroCrossPolarity(void)
{
    return (nextPhase & 0x01);
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

	ADMUXTable[0] = ADMUX_COIL_C;
	ADMUXTable[1] = ADMUX_COIL_B;
	ADMUXTable[2] = ADMUX_COIL_A;
	ADMUXTable[3] = ADMUX_COIL_C;
	ADMUXTable[4] = ADMUX_COIL_B;
	ADMUXTable[5] = ADMUX_COIL_A;

	currentTable[0] = ADMUX_CURR_A;
	currentTable[1] = ADMUX_CURR_A;
	currentTable[2] = ADMUX_CURR_B;
	currentTable[3] = ADMUX_CURR_B;
	currentTable[4] = ADMUX_CURR_C;
	currentTable[5] = ADMUX_CURR_C;

	startupDelays[0] = 1000;
	startupDelays[1] = 700;
	startupDelays[2] = 500;
	startupDelays[3] = 500;
	startupDelays[4] = 300;
	startupDelays[5] = 300;
}