#include "./include/interrupts.h"
#include "./include/functions.h"
#include "./include/serial.h"
#include <util/delay.h>

void testADC()
{
    while (1)
    {
        debug_print(readChannel(ADMUX_SPD_REF), "Speedref = ");
        startupDelay(500);
    }

}


int main(void)
{
    uart_init(9600);
    initPorts();
    initTimers();
    generateTables();
    initADC();
    //testADC();
    startMotor();

    while(1)
    {
        PORTD |= (1 << PD4);
        startupDelay(500);
        PORTD &= ~(1 << PD4);
        startupDelay(500);
    }
    return 0;
}
