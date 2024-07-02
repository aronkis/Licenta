#include "./include/interrupts.h"
#include "./include/functions.h"
#include "./include/serial.h"

int main(void)
{
    setup();

    while(TRUE)
    { 
        runMotor();
    }
}
