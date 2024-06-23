#include "./include/interrupts.h"
#include "./include/functions.h"
#include "./include/serial.h"
#include <util/delay.h>


int main(void)
{
    uart_init(19200);
    initPorts();
    initTimers();
    generateTables();
    initADC();
    if (!debugMode)
    {
        startMotor();
    }
    else
    {
        uart_send_string("Debug Mode.");
        while(1)
        {
            GREEN_LED;
            _delay_ms(500);
            RED_LED;
            _delay_ms(500);
        }
    }
    GREEN_LED;
    while(1)
    { 
        debug_print(OCR1A, "OCR1A=");
        debug_print((714285 / thirtyDegreeTime), "spd=");
        _delay_ms(250);
        // for (int i = 0; i < NUMBER_OF_STEPS; i++)
        // {
        //     debug_print(sixtyDegreeTimes[i], "sxt=");
        // }
        // debug_print(OCR1A, "OCR1A=");
        // if (adcInt)
        // {
        //     debug_print(speedRef, "spdr=");
        //     debug_print(OCR0B, "OCR0B=");
        // }
        // uart_send_string("\n\r");
    }
    RED_LED;
    return 0;
}
