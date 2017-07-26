#ifndef _MAIN_CONFIG_H
#define _MAIN_CONFIG_H

#include "stm32f10x.h"
#include "bsp.h"





//*****************************************************  choose board
#define BOARD_MAIN_CONTROLLER_TYPE 0
#define BOARD_NAVIGATION_TYPE 0
#define BOARD_MONITOR_TYPE 0

#define BOARD_TREX_TYPE 1








//****************************************************¡¡config board pin

#if  BOARD_NAVIGATION_TYPE

#define UARTDEV		 	USART1
#define UART_TX_GPIO 	GPIOB
#define UART_TX_PIN 	GPIO_Pin_6
#define UART_RX_GPIO 	GPIOB
#define UART_RX_PIN 	GPIO_Pin_7
#define UART_PIN_REMAP_FUNC() GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE)
#define UART_BAUDRATE	9600
#define CAN1_ID			0x11
#define DEF_MAIN_CONTROLLER_CAN1_ID 0X0
#define IAP_PORT_UART 1
#define IAP_PORT_CAN1 0
#endif




#if BOARD_MONITOR_TYPE
#define UARTDEV		 	USART3
#define UART_TX_GPIO 	GPIOB
#define UART_TX_PIN 	GPIO_Pin_10
#define UART_RX_GPIO 	GPIOB
#define UART_RX_PIN 	GPIO_Pin_11
#define UART_PIN_REMAP_FUNC() GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE)
#define UART_BAUDRATE	9600
#define CAN1_ID			0x16
#define DEF_MAIN_CONTROLLER_CAN1_ID 0x10
#define IAP_PORT_UART 1
#define IAP_PORT_CAN1 0
#endif

#if BOARD_TREX_TYPE
#define IAP_PORT_UART 1
#define IAP_PORT_CAN1 0
//uart config , if IAP_PORT_UART==0 , do not care these 
#define UARTDEV		 	USART2
#define UART_TX_GPIO 	GPIOA
#define UART_TX_PIN 	GPIO_Pin_2
#define UART_RX_GPIO 	GPIOA
#define UART_RX_PIN 	GPIO_Pin_3
#define UART_PIN_REMAP_FUNC() //GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE)
#define UART_BAUDRATE	57600
//can1 config , if IAP_PORT_CAN1==0 , do not care these 
#define CAN1_ID			0x11
#define DEF_MAIN_CONTROLLER_CAN1_ID 0X0
#endif



//**********************************************************  config board hse clock

//config hse in "Options for Target" 
















//******************************************************** config iap 

#include "iap.h"









#endif
