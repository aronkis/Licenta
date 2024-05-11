#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdint.h>

#define RX_BUFFER_SIZE 128

void uart_init(uint32_t baud);
void uart_send_byte(const char c);
void uart_send_array(const char *c,uint16_t len);
void uart_send_string(const char *c);
uint16_t uart_read_count(void);
char uart_read(void);
void debug_print(unsigned int number, const char *text);

#endif