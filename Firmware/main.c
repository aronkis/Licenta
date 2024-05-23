#include "./include/interrupts.h"
#include "./include/functions.h"
#include "./include/serial.h"
#include <util/delay.h>


int main(void)
{
    //uart_init(19200);
    initPorts();
    initTimers();
    generateTables();
    startMotor();
    
    while(1)
    {
        PORTD |= (1 << PD4);
        startupDelay(250);
        PORTD &= ~(1 << PD4);
        startupDelay(250);
    }
    return 0;
}
