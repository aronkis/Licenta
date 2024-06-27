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
            case STARTUP:
                startMotor();
            break;
            case RUNNING: 
                debugPrint((0xAE62D / thirtyDegreeTime), "spd=");
                debugPrint(OCR0B, "pwm=");
                checkForMotorStop();
                
            break;
            case RESTART: 
                stopMotor();
                checkForStartMotor();
            break;
        }
    }
    RED_LED;
}
