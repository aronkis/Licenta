#include <stdio.h>
#include <avr/interrupt.h>
#include "../include/serial.h"
#include "../include/functions.h"

volatile static char RXBuffer[RX_BUFFER_SIZE] = { 0 };
volatile static uint16_t RXCount = 0;	
volatile static uint8_t uartTXBusy = 1;

void USART_RX_vect(void)
{
	volatile static uint16_t RXWritePos = 0;
	
	RXBuffer[RXWritePos] = UDR0;
	RXCount++;
	RXWritePos++;
	if (RXWritePos >= RX_BUFFER_SIZE)
	{
		RXWritePos = 0;
	}
}

void USART_TX_vect(void)
{
	uartTXBusy = 1;
}

void uartInit(uint32_t baud)
{
	static uint8_t speed = 16;
	
	baud = (F_CPU/(speed*baud)) - 1;
	
	UBRR0H = (baud & 0x0F00) >> 8;
	UBRR0L = (baud & 0x00FF);
	
	UCSR0B |= SET_BIT(TXEN0) | SET_BIT(RXEN0) |
		      SET_BIT(TXCIE0) | SET_BIT(RXCIE0);
	sei();
}

void uartSendBtye(const char c)
{
	while(uartTXBusy == 0);
	uartTXBusy = 0;
	UDR0 = c;
}

void uartSendArray(const char *c, uint16_t len)
{
	for(uint16_t i = 0; i < len; i++){
		uartSendBtye(c[i]);
	}
}

void uartSendString(const char *c)
{
	uint16_t i = 0;
	
	do {
		uartSendBtye(c[i]);
		i++;
	} while(c[i] != '\0');

	uartSendBtye(c[i]);
}

uint16_t uartReadCount(void)
{
	return RXCount;
}

char uartRead(void)
{
	static uint16_t RXReadPos = 0;
	char data = 0;
	
	data = RXBuffer[RXReadPos];
	RXReadPos++;
	RXCount--;
	
	if(RXReadPos >= RX_BUFFER_SIZE)
	{
		RXReadPos = 0;
	}
	
	return data;
}

void debugPrint(unsigned int number, const char *text)
{
	char numberStr[6];
	sprintf(numberStr, "%d", number);
	uartSendString(text);
	uartSendString(numberStr);
	uartSendString("\n\r");
}