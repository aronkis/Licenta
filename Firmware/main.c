#include "./include/interrupts.h"
#include "./include/functions.h"
#include "./include/serial.h"

int main(void)
{
    uartInit(19200);
    initPorts();
    initTimers();
    generateTables();
    initADC();
    GREEN_LED;
    while(TRUE)
    { 
        switch (programState)
        {
            case 0: // startup
                startMotor();
            break;
            case 1: // running
                debugPrint((714285 / thirtyDegreeTime), "spd=");
                debugPrint(OCR0B, "ocr0b=");
                checkForMotorStop();
            break;
            case 2: // stop/restart
                stopMotor();
                checkForStartMotor();
            break;
        }
    }
    RED_LED;
    return 0;
}
