#include "./include/interrupts.h"
#include "./include/functions.h"
#include "./include/serial.h"
#include <util/delay.h>

volatile uint8_t programState = 0;
volatile uint16_t motorStopCounter;


int main(void)
{
    uart_init(19200);
    initPorts();
    initTimers();
    generateTables();
    initADC();
    if (debugMode)
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
        switch (programState)
        {
        case 0: // starting the motor
            startMotor();   
            programState = 1;         
        break;
        case 1: //sending data for visualization
            // debug_print(OCR1A, "OCR1A=");
            // debug_print((714285 / thirtyDegreeTime), "spd=");
            // debug_print(speedRef, "spdref = ");
            debug_print(motorStopCounter, "msc = ");
            // _delay_ms(250);
        break;
        case 2: // stop the motor and wait for the voltage ref to increase
            uart_send_string("ProgramState == 2");
            stopMotor();
            _delay_ms(15);
            initADC();
            programState = 0;
        break;
        default:
            break;
        }
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
