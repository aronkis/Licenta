//#define DEBUG_RUN
#define NORMAL_RUN
#ifdef NORMAL_RUN
    #include "./include/interrupts.h"
#endif

#ifdef DEBUG_RUN
    #include <avr/io.h>
#endif

#include "./include/functions.h"
#include "./include/serial.h"

//TODO: Implement Closed Loop

int main(void)
{
    uart_init(57600);
    uart_send_string("STARTING\n\r");
    sei();
    #ifdef NORMAL_RUN
        initPorts();
        uart_send_string("INIT_PORTS_DONE\n\r");
        initTimers();
        uart_send_string("INIT_TIMERS_DONE\n\r");
        #ifdef ADC_MEASURE
            initADC();
            uart_send_string("INIT_ADC_DONE\n\r");
        #endif
        initComparator();
        uart_send_string("INIT_COMPARATOR_DONE\n\r");
        generateTables();
        uart_send_string("TABELES_GENERATED\n\r");
        startMotor();
       // enableWatchdogTimer();
       // sei();
        while (1)
        {
            if(speedUpdated)
            {
                //uart_send_string("Speed updated\n\r");
                speedUpdated = FALSE;
                SET_COMPB_TRIGGER_VALUE(PWM_START_VALUE);
            }
            else
            {
                //uart_send_string("Speed not updated\n\r");
            }
        }
    #endif

    #ifdef DEBUG_RUN
        DDRB = 0;
        DDRB = (1 << PB5);
        PORTB = 0x00;
        TCCR1B |= SET_BIT(CS11);
        while (1)
        {
            uart_send_string("TEST\n\r");
            startupDelay(25000);
            PORTB |= SET_BIT(PB5);
            debug_print(PORTB, "PORTB = ");
            startupDelay(25000);
            PORTB = 0;
            debug_print(PORTB, "PORTB = ");
        }
    #endif

    return 0;
}
