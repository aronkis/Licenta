#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_

#define PWM_IN_MIN 1000
#define PWM_IN_MAX 2000
#define PWM_MIN_VALUE 35
#define PWM_MAX_VALUE 250
#define PWM_START_VALUE 88

static volatile byte last_PWM_state   	  = 0, // 0 PWM low, 1 PWM high
     				 last_timer_state     = 0, // 0 PIN ON, 1 PIN OFF
	 				 timer_value;
static volatile unsigned long pwm_average = 0;
static volatile unsigned long temp = 0;
static volatile unsigned int pwm_input = 0;
static volatile int timer_overflow_counter = 0;
static volatile byte current_phase = 0;
static volatile byte current_highside;
byte count = 0;


void set_next_step();
void set_timer(byte);

// Used to check the zero crossing (5 times to eliminate noise)
ISR(ANALOG_COMP_vect)
{
	set_next_step();
	current_phase++;
	if (current_phase >= 6)
		current_phase = 0;
}

ISR(PCINT0_vect)
{
	if (!last_PWM_state)
	{
		last_PWM_state = 1;
		timer_overflow_counter = 0;
		TCCR2B |= (1 << CS20); 
		TCNT2 = 0;
	}
	else if (last_PWM_state)
	{
		pwm_input = TCNT2 + (timer_overflow_counter * 255 / 16);
		temp += pwm_input;
		count++;
		if (count == 100) 
		{
			pwm_average = temp / 100;
			pwm_average = constrain(pwm_average, PWM_IN_MIN, PWM_IN_MAX);
			timer_value = map(pwm_average, PWM_IN_MIN, PWM_IN_MAX, \
										   PWM_MIN_VALUE, PWM_MAX_VALUE);
			
			set_timer(timer_value);
			count = 0;
			temp = 0;
		}
		TCCR2B = 0;
		last_PWM_state = 0;
	}
}

ISR(TIMER2_OVF_vect)
{
	timer_overflow_counter++;
}

ISR(TIMER1_COMPA_vect)
{
	if (last_timer_state)
	{
		PORTB &= ~PORTB;
		last_timer_state = 0;
	}
	else
	{
		PORTB |= (1 << current_highside);
		last_timer_state = 1;
	}
}

ISR(TIMER1_OVF_vect)
{
	last_timer_state = 1;
}

#endif // _INTERRUPTS_H_