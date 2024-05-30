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
    RED_LED;
    startMotor();
    while(1) 
    { 
        if (adcInt)
        {
            debug_print(adcRead, "ar=");
            adcInt = FALSE;
        }
        if (adcFlag)
        {
            // for (int i = 0; i < NUMBER_OF_STEPS; i++)
        	// {
            // 	debug_print(sixtyDegreeTimes[i], "sixtyDegreeTimesFlag[i]=");
        	// }
            debug_print(adcRead, "zcar=");
            debug_print(ZC_DETECTION_THRESHOLD, "zcth=");
            debug_print(thirtyDegreeTimesave, "tdtas=");
            debug_print(thirtyDegreeTime, "tdta=");
            
            adcFlag = FALSE;
        }   
        if (compFlag)
        {
            uart_send_string("comp\n\r");
			debug_print(nextPhase, "npcpf=");
            compFlag = FALSE;
        }
    }
    RED_LED;
    return 0;
}
