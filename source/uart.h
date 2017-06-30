#ifndef UART_H_
#define UART_H_

#include "stm32f10x.h"
#include "fifo.h"
#include  <ctype.h>
#include  <string.h>
#include  <stdio.h>	
#include "main_config.h"
#include "systick.h"




#define UART_BUFFER_LEN 256

typedef void (*uartReadCallBack)(char c);

typedef struct _uart_t {
	USART_TypeDef *uartDev;
	uartReadCallBack read_cb;
	fifo_t txfifo;
	fifo_t rxfifo;
	char txbuff[UART_BUFFER_LEN];
	char rxbuff[UART_BUFFER_LEN];
}Uart_t;


void Uart_Configuration (Uart_t *uart, USART_TypeDef *uartDev, uint32_t USART_BaudRate, uint16_t USART_WordLength, uint16_t USART_StopBits, uint16_t USART_Parity);
void Uart_DeInit (Uart_t *uart);
int Uart_Get(Uart_t *uart, unsigned char *buffer, int count);
void Uart_Put(Uart_t *uart,unsigned char *data, int count);



#endif
