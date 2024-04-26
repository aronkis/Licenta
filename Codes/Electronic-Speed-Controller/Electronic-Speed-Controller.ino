#include "./include/includes.h"

static volatile int motor_off_counter = 0;
static volatile int soft_starter_countdown = 0;
static volatile int motor_state = 0;

void setup()
{
	Serial.begin(115200);
	Serial.println("START");
	startupDelays[0] = 200;
	startupDelays[1] = 150;
	startupDelays[2] = 100;
	startupDelays[3] = 80;
	startupDelays[4] = 70;
	startupDelays[5] = 65;
	startupDelays[6] = 60;
	startupDelays[7] = 55;
		
	set_up_ports();

	// // Timer1 is used to toggle the high side pins
	set_up_timer1();

	// // Analog comparator setting
	ACSR = (1 << ACI);

	// Timer2 is used to measure the incoming PWM signal
	//set_up_timer2();	
} 

void loop()
{
	pwm_average = 1250;
	//TODO : Measure the PWM starting point
	if (pwm_average > (PWM_IN_MIN + 200))
	{
		motor_off_counter = 0;
		motor_state = 1;
	}	

	if (motor_state)
	{
		start_motor();

		while (motor_state)
		{
			set_timer(88);
		}
	}
}
