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
        debug_print((714285 / thirtyDegreeTime), "spd=");
        debug_print(OCR0B, "ocr0b=");
    }
    RED_LED;
    return 0;
}
