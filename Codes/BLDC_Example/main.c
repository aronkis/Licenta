/*
bldc sensorless driver sample

copyright (c) Davide Gironi, 2013

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "adc/adc.h"

#define UART_BAUD_RATE 9600
#include "uart/uart.h"

#include "bldcsensorless/bldcsensorless.h"


//speed pot channel
#define POTSPEED_CHANNEL 3

//direction switch
#define BUTTONDIRECTION_DDR DDRC
#define BUTTONDIRECTION_PIN PINC
#define BUTTONDIRECTION_INPUT PC5

int main (void) {
	//init uart
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );

	//init adc
	adc_init();

	//init interrupt
	sei();

	//init bldc driver
	bldcsensorless_init();

	//direction and speed update interval counter
	uint16_t controlupd_interval = 0;

	//init potspeed and direction button
	uint16_t potspeed = 0;
	uint16_t potspeedold = 0;
	BUTTONDIRECTION_DDR &= ~(1<<BUTTONDIRECTION_INPUT);
	uint16_t direction = 0;
	uint16_t directionold = 0;

	//int speed and direction
    bldcsensorless_setspeed(100);
    bldcsensorless_setdirection(BLDCSENSORLESS_DIRECTIONCW);

	//start motor spin
	bldcsensorless_setstart();

	#if BLDCSENSORLESS_DEBUG == 1
    char printbuff[100];
	#endif

	for(;;) {

		//do not check check for update every cycles
		controlupd_interval++;
		if(controlupd_interval > 400) {
			controlupd_interval = 0;

			//check for direction
			if(BUTTONDIRECTION_PIN & (1<<BUTTONDIRECTION_INPUT)) {
				direction = BLDCSENSORLESS_DIRECTIONCW;
			} else {
				direction = BLDCSENSORLESS_DIRECTIONCCW;
			}
			if(direction != directionold) { //direction changed
				//stop motor
				bldcsensorless_setstop();
				//check for speed (any adc read must be done when the motor is stop)
				potspeed = adc_read(POTSPEED_CHANNEL); //read filtered pot speed
				if(potspeed != potspeedold) {
					potspeedold = potspeed;
					uint16_t potspeedn = (long)(potspeed - 0) * (long)(100 - 0) / (long)(1024 - 0) + 0;
					bldcsensorless_setspeed(potspeedn);
				}
				//wait for the motor to stop
				_delay_ms(2000);
				//set direction
				if(direction == BLDCSENSORLESS_DIRECTIONCW) {
					uart_puts("direction changed to: cw\r\n");
				} else if(direction == BLDCSENSORLESS_DIRECTIONCCW) {
					uart_puts("direction changed to: ccw\r\n");
				}
				directionold = direction;
				bldcsensorless_setdirection(direction);
				//start pmotor
				bldcsensorless_startupmotor();
				bldcsensorless_setstart();
			}

		}

	}
}

