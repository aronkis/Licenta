#include "./include/interrupts.h"
#include "./include/functions.h"
#include "./include/serial.h"
#include <util/delay.h>

uint16_t count2 = 0;

int main(void)
{
    uart_init(19200);
    initPorts();
    initTimers();
    generateTables();
    initADC();
    RED_LED;
    thirtyDegreeTimesave = 2000;
    startMotor();
    while(1)
    { 
       
    //     // if (adcInt)
    //     // {
    //     //     debug_print(adcRead, "ar=");
    //     //     adcInt = FALSE;
    //     // }
        if (adcInt) 
        {
            count2++;
            adcInt = FALSE;
        }
        if (count2 == 60)
        {
            // for (int i = 0; i < NUMBER_OF_STEPS; i++)
        	// {
            // 	debug_print(sixtyDegreeTimes[i], "sixtyDegreeTimesFlag[i]=");
        	// }
            // uart_send_string("\n\r");
            if (thirtyDegreeTimesave > 60)
            {
                thirtyDegreeTimesave -= 10;
                debug_print(thirtyDegreeTimesave, "tdts=");
            }
            count2 = 0;
        }
        // {
            // for (int i = 0; i < NUMBER_OF_STEPS; i++)
        	// {
            // 	debug_print(sixtyDegreeTimes[i], "sixtyDegreeTimesFlag[i]=");
        	// }
            // debug_print(thirtyDegreeTime12, "tdta=");
            // debug_print(thirtyDegreeTimesave, "tdtas=");
            // debug_print(TCNT1Save, "tcnt1s=");
            // debug_print(adcRead, "zcar=");
            // debug_print(ZC_DETECTION_THRESHOLD, "zcth=");
            // uart_send_string("\n\r");
            // adcInt = FALSE;
        	// SET_COMPx_TRIGGER_VALUE(OCR0B, speedRef);
        // }   
        // if (adcInt == 2)
        // {
        //     debug_print(speedRef, "spd_ref=");
        //     debug_print(speedRefMap, "spdrefmap=");
        //     uart_send_string("\n\r");
        //     adcInt = 0;
        // }
        // if (compFlag)
        // {
        //     uart_send_string("comp\n\r");
		// 	debug_print(nextPhase, "npcpf=");
        //     uart_send_string("\n\r");
        //     compFlag = FALSE;
        // }
    }
    RED_LED;
    return 0;
}
