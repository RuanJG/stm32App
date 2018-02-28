#ifndef UART_H_
#define UART_H_

#include "fifo.h"
#include  <ctype.h>
#include  <string.h>
#include  <stdio.h>	





#define UART_BUFFER_LEN 128

typedef void (*uartReadCallBack)(unsigned char c);

typedef struct _uart_t {
	USART_TypeDef *uartDev;
	uartReadCallBack read_cb;
	fifo_t txfifo;
	fifo_t rxfifo;
	unsigned char txbuff[UART_BUFFER_LEN];
	unsigned char rxbuff[UART_BUFFER_LEN];
}Uart_t;


void Uart_Configuration (Uart_t *uart, USART_TypeDef *uartDev, uint32_t USART_BaudRate, uint16_t USART_WordLength, uint16_t USART_StopBits, uint16_t USART_Parity);
void Uart_DeInit (Uart_t *uart);
int Uart_Get(Uart_t *uart, unsigned char *buffer, int count);
void Uart_Put(Uart_t *uart,unsigned char *data, int count);
void Uart_Clear_Rx( Uart_t *uart );
void Uart_Clear_Tx( Uart_t *uart );
void Uart_Put_Sync(Uart_t *uart,unsigned char *data, int count);

#endif
