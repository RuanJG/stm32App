/******************************************************************************
 * @file:    SetUART.c
 * @purpose: functions related to UART
 * @version: V1.00
 * @date:    11. Jul 2011
 *----------------------------------------------------------------------------
 ******************************************************************************/

#include "stm32f10x.h"
#include "fifo.h"
#include  <ctype.h>
#include  <string.h>
#include  <stdio.h>	
#include "main_config.h"
#include "systick.h"
#include "uart.h"

Uart_t *uart1_p ;
Uart_t *uart2_p ;
Uart_t *uart3_p ;

void Uart_Configuration (Uart_t *uart, USART_TypeDef *uartDev, uint32_t USART_BaudRate, uint16_t USART_WordLength, uint16_t USART_StopBits, uint16_t USART_Parity)
{		  
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//softwoare config
	fifo_init(&uart->txfifo, uart->txbuff, sizeof(char), sizeof(uart->txbuff));
	fifo_init(&uart->rxfifo, uart->rxbuff, sizeof(char), sizeof(uart->rxbuff));
	
	// pll clock
	if( uartDev == USART1 ){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
	}else if( uartDev == USART2 ){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 	
	}else if( uartDev == USART3 ){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	}
	
	// uart base config
	USART_ITConfig(uartDev, USART_IT_RXNE, DISABLE);
	USART_ITConfig(uartDev, USART_IT_TXE, DISABLE);
	USART_Cmd(uartDev, DISABLE);

	USART_StructInit (&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = USART_BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength;
	USART_InitStructure.USART_StopBits = USART_StopBits;
	USART_InitStructure.USART_Parity = USART_Parity;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
	USART_Init(uartDev, &USART_InitStructure); 


	//user cant set this read_cb by youself, example , in Iap_jumper 
	uart->read_cb = NULL;
	uart->uartDev = uartDev;
	
	// irq
	if( uartDev == USART1 ){
		uart1_p = uart;
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CUSTOM_UART1_IRQ_PREPRIORITY;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = CUSTOM_UART1_IRQ_SUBPRIORITY;
	}else if( uartDev == USART2 ){
		uart2_p = uart;
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CUSTOM_UART2_IRQ_PREPRIORITY;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = CUSTOM_UART2_IRQ_SUBPRIORITY;
	}else if( uartDev == USART3 ){
		uart3_p = uart;
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CUSTOM_UART3_IRQ_PREPRIORITY;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = CUSTOM_UART3_IRQ_SUBPRIORITY;
	}
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	
	
	

	// start .... 
	USART_ITConfig(uartDev, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(uartDev, USART_IT_TXE, ENABLE);
	USART_Cmd(uartDev, ENABLE);	  
	/*
	@TODO check this loop is need ?
	while (USART_GetFlagStatus(uartDev, USART_FLAG_TC) == RESET)
	{
	}	
	*/
}

void Uart_DeInit (Uart_t *uart)
{
	USART_ITConfig(uart->uartDev, USART_IT_RXNE, DISABLE);
	USART_ITConfig(uart->uartDev, USART_IT_TXE, DISABLE);
	USART_Cmd(uart->uartDev, DISABLE);	
}

void _uart_irq_function(Uart_t *uart)
{
	unsigned char c,tc;
	USART_TypeDef *uartDev = uart->uartDev;
	
	if(USART_GetITStatus(uartDev, USART_IT_RXNE) != RESET)  	//Rec Interrupt enabled
	{
		if( USART_GetFlagStatus(uartDev, USART_FLAG_RXNE) != RESET) // data recived
		{
			c=USART_ReceiveData(uartDev);
			fifo_put_force(&uart->rxfifo,c);
			if( uart->read_cb != NULL)
				uart->read_cb(c);
			
			//if( uartDev == USART3 )
				//USART_SendData(USART1, tc);
		}
	}

	if(USART_GetITStatus(uartDev, USART_IT_TXE) != RESET)	//�����ж�enabled
	{   
		//if( USART_GetFlagStatus(uartDev, USART_FLAG_TXE) != RESET)
		//{

			if( 0 < fifo_get(&uart->txfifo,&tc) ){
				USART_SendData(uartDev, tc);
				//if( uartDev == USART3 )
					//USART_SendData(USART1, tc);
			}else{
				USART_ITConfig(uartDev, USART_IT_TXE, DISABLE);
			}
			
		//}
	}
}


void USART1_IRQHandler(void)
{
	_uart_irq_function(uart1_p);
} 

void USART2_IRQHandler(void)
{
	_uart_irq_function(uart2_p);
} 

void USART3_IRQHandler(void)
{
	_uart_irq_function(uart3_p);
}



int Uart_Get(Uart_t *uart, unsigned char *buffer, int count){
	// return the count of the data filled in buffer from uart fifo 
	int i;
	
	if( uart == NULL )
		return 0;
	
	USART_ITConfig(uart->uartDev, USART_IT_RXNE, DISABLE);
	for( i=0; i< count; i++){
		if( 0 == fifo_get(&uart->rxfifo, buffer+i ) ){
			break;
		}
	}
	USART_ITConfig(uart->uartDev, USART_IT_RXNE, ENABLE);

	return i;
}


void Uart_Put(Uart_t *uart,unsigned char *data, int count)
{
	int i;
	USART_TypeDef *uartDev;
	
	if( uart == NULL )
		return;
	
	uartDev = uart->uartDev;
	
	USART_ITConfig(uartDev, USART_IT_TXE, DISABLE);
	
	for( i=0; i<count; i++){
		if( 0 == fifo_put( &uart->txfifo , data[i] ) ){
			//fifo is full
			USART_ITConfig(uartDev, USART_IT_TXE, ENABLE);
			while( 0 == fifo_put( &uart->txfifo , data[i] ) )
				;//systick_delay_us(100);
			USART_ITConfig(uartDev, USART_IT_TXE, DISABLE);
		}
	}
	
	USART_ITConfig(uartDev, USART_IT_TXE, ENABLE);
}


void Uart_Put_Sync(Uart_t *uart,unsigned char *data, int count)
{
	int i;
	USART_TypeDef *uartDev;
	
	if( uart == NULL )
		return;
	
	uartDev = uart->uartDev;
	
	USART_ITConfig(uartDev, USART_IT_TXE, DISABLE);
	
	for( i=0; i<count; i++){
		while( USART_GetFlagStatus(uartDev, USART_FLAG_TXE) == RESET );
		USART_SendData(uartDev, data[i]);
	}
	
	USART_ITConfig(uartDev, USART_IT_TXE, ENABLE);
}

void Uart_Clear_Rx( Uart_t *uart )
{
	if( uart == NULL )
		return;
	
	USART_ITConfig(uart->uartDev, USART_IT_RXNE, DISABLE);
	
	fifo_clear( &uart->rxfifo );
	
	USART_ITConfig(uart->uartDev, USART_IT_RXNE, ENABLE);
}

void Uart_Clear_Tx( Uart_t *uart )
{
	if( uart == NULL )
		return;
	
	USART_ITConfig(uart->uartDev, USART_IT_TXE, DISABLE);
	
	fifo_clear( &uart->txfifo );
	
	USART_ITConfig(uart->uartDev, USART_IT_TXE, ENABLE);
}



Uart_t *pConsoleUart = NULL;


void Uart_config_console( Uart_t* u )
{
	//TODO lock ?
	
	pConsoleUart = u;
}



int fputc(int ch, FILE *f)
{
	//TODO lock ?
	
	if( pConsoleUart != NULL ){
		Uart_Put( pConsoleUart, (unsigned char*)&ch, 1);
	}
	return (ch);
}


//End of File
