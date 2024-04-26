#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#define RISING  ((1 << ACIS0) | (1 << ACIS1))
#define FALLING  (1 << ACIS1)

#define ADC_PIN_B 0
#define ADC_PIN_C 1
#define ADC_PIN_A 2

#define AH PB3
#define BH PB2
#define CH PB1

#define AL PD4
#define BL PD3
#define CL PD2

#define AH_BL 0
#define AH_CL 1
#define BH_CL 2
#define BH_AL 3
#define CH_AL 4
#define CH_BL 5

#define START_UP_COMMS 8

unsigned int startupDelays[START_UP_COMMS];

void set_up_timer1()
{
	TCCR1A = (1 << WGM10);					// PWM, Phase correct, 8-bit
	TCCR1B = (1 << CS10);					// Set clock source to clkI/O / 1 (no prescaling)
	TIMSK1 = (1 << TOIE1) | (1 << OCIE1A); // Enable timer1 interrupt overflow and compare match A interrupt
	TIFR1  = (1 << OCF1A);					// Output compare A match flag
}

void set_up_timer2()
{
	TCCR2A  = 0;
	TCCR2B  = 0;
	TIMSK2  = (1 << TOIE2); // Enable timer2 interrupt overflow

	// Enabling pin change interrupt on PB0
	//PCICR   = (1 << PCIE0);	 // Enable interrupts on PCINT[0:7]
	//PCMSK0  = (1 << PCINT0); 
}

void set_up_ports()
{
	// Configure pins 2, 3 and 4 as outputs
	DDRD   = (1 << AL) | (1 << BL) | (1 << CL); 
	PORTD &= 0x00;								  
	// Configure pins 1, 2 and 3 as outputs
	DDRB   = (1 << AH) | (1 << BH) | (1 << CH); 
	PORTB &= 0x00;		
}

void set_up_comparator()
{
	ADCSRA &= ~(1 << ADEN); // Disable the ADC module
	ADCSRB |= (1 << ACME); // Enable MUX select for negative input of comparator
	ACSR |= (1 << ACIE);  // Enable the analog comparator interrupt
	ACSR |= (1 << ACI);   // Clear the analog comparator interrupt
}

inline static void set_timer(byte timer_value)
{
	OCR1A = timer_value;
}

void mosfet_state (byte high_side, byte low_side)
{
	PORTD &= ~PORTD;
	PORTB &= ~PORTB;
	PORTD |= (1 << low_side);
	current_highside = high_side; // Used to tell which mosfet is switching
}

void bemf_sensing(byte adc_pin, byte bemf_direction)
{
	ADMUX = adc_pin;
	//ACSR &= ~(1 << ACIE); // Disabling the comparator interrupt flag, to avoid unwanted interrupts on mode select
	ACSR &= ~((1 << ACIS0) | (1 << ACIS1));
	ACSR |= bemf_direction;
	//ACSR |= (1 << ACIE);
}

void soft_start_next_step()
{
	Serial.println(current_phase);

	switch (current_phase)
	{
	case AH_BL:
		mosfet_state(AH, BL);
		break;
	case AH_CL:
		mosfet_state(AH, CL);
		break;
	case BH_CL:
		mosfet_state(BH, CL);
		break;
	case BH_AL:
		mosfet_state(BH, AL);
		break;
	case CH_AL:
		mosfet_state(CH, AL);
		break;
	case CH_BL:
		mosfet_state(CH, BL);
		break;
	}
}

void set_next_step()
{
	Serial.println(current_phase);

	switch (current_phase)
	{
	case AH_BL:
		mosfet_state(AH, BL);
		bemf_sensing(ADC_PIN_C, RISING);
		break;
	case AH_CL:
		mosfet_state(AH, CL);
		bemf_sensing(ADC_PIN_B, FALLING);
		break;
	case BH_CL:
		mosfet_state(BH, CL);
		bemf_sensing(ADC_PIN_A, RISING);
		break;
	case BH_AL:
		mosfet_state(BH, AL);
		bemf_sensing(ADC_PIN_C, FALLING);
		break;
	case CH_AL:
		mosfet_state(CH, AL);
		bemf_sensing(ADC_PIN_B, RISING);
		break;
	case CH_BL:
		mosfet_state(CH, BL);
		bemf_sensing(ADC_PIN_A, FALLING);
		break;
	}
}




void start_motor()
{
	unsigned char i;

	set_timer(PWM_START_VALUE);
	current_phase = 0;

	soft_start_next_step();

	delay(1000);

	current_phase++;

	for (i = 0; i < START_UP_COMMS; i++)
	{
		soft_start_next_step();
		delay(startupDelays[i]);

		current_phase++;
		if (current_phase >= 6)
			current_phase = 0;
	}
	set_up_comparator();

}

#endif // _FUNCTIONS_H_