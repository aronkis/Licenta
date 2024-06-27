#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdint.h>

#define RX_BUFFER_SIZE 128

void __attribute__((__signal__))
     __attribute__((__used__))
USART_RX_vect(void);

void __attribute__((__signal__))
     __attribute__((__used__))
USART_TX_vect(void);

void uartInit(uint32_t baud);
void uartSendBtye(const char c);
void uartSendArray(const char *c, uint16_t len);
void uartSendString(const char *c);
void debugPrint(unsigned int number, const char *text);
char uartRead(void);
uint16_t uartReadCount(void);

#endif // _SERIAL_H_