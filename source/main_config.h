#ifndef _MAIN_CONFIG_H
#define _MAIN_CONFIG_H

#include "stm32f10x.h"
#include "bsp.h"


#define CUSTOM_CAN1_IRQ_PREPRIORITY 2
#define CUSTOM_CAN1_IRQ_SUBPRIORITY 0


//*****************************************************  choose board
#define COMMON_BOARD_TYPE 0
#define BOARD_TREX_V1_TYPE 0
#define BOARD_TREX_V2_TYPE 1



#if COMMON_BOARD_TYPE

// IAP uart config
#define IAP_PORT_UART 0
#define UARTDEV		 	USART1
#define UART_TX_GPIO 	GPIOA
#define UART_TX_PIN 	GPIO_Pin_9
#define UART_RX_GPIO 	GPIOA
#define UART_RX_PIN 	GPIO_Pin_10
#define UART_PIN_REMAP_FUNC() //GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE)
#define UART_BAUDRATE	57600

//IAP Can1 Config
#define IAP_PORT_CAN1 0
#define CAN1_ID			0x11
#define DEF_MAIN_CONTROLLER_CAN1_ID 0X0

//IAP USB config
#define IAP_PORT_USB 1

//IAP GPIO detection Config
#define IAP_GPIO_DETECTION 1
#define IAP_GPIO GPIOB
#define IAP_GPIO_PIN GPIO_Pin_2
#define IAP_GPIO_LEVEL 1

#endif  // COMMON_BOARD_TYPE


#if BOARD_TREX_V2_TYPE

// IAP uart config
#define IAP_PORT_UART 1
#define UARTDEV		 	USART3
#define UART_TX_GPIO 	GPIOB
#define UART_TX_PIN 	GPIO_Pin_10
#define UART_RX_GPIO 	GPIOB
#define UART_RX_PIN 	GPIO_Pin_11
#define UART_PIN_REMAP_FUNC() //GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE)
#define UART_BAUDRATE	57600

//IAP Can1 Config
#define IAP_PORT_CAN1 0
#define CAN1_ID			0x11
#define DEF_MAIN_CONTROLLER_CAN1_ID 0X0

//IAP USB config
#define IAP_PORT_USB 1

//IAP GPIO detection Config
#define IAP_GPIO_DETECTION 1
#define IAP_GPIO GPIOB
#define IAP_GPIO_PIN GPIO_Pin_2
#define IAP_GPIO_LEVEL 1

#endif  // BOARD_TREX_V2_TYPE


#if BOARD_TREX_V1_TYPE

// IAP uart config
#define IAP_PORT_UART 1
#define UARTDEV		 	USART1
#define UART_TX_GPIO 	GPIOA
#define UART_TX_PIN 	GPIO_Pin_9
#define UART_RX_GPIO 	GPIOA
#define UART_RX_PIN 	GPIO_Pin_10
#define UART_PIN_REMAP_FUNC() //GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE)
#define UART_BAUDRATE	57600

//IAP Can1 Config
#define IAP_PORT_CAN1 0
#define CAN1_ID			0x11
#define DEF_MAIN_CONTROLLER_CAN1_ID 0X0

//IAP USB config
#define IAP_PORT_USB 1

//IAP GPIO detection Config
#define IAP_GPIO_DETECTION 1
#define IAP_GPIO GPIOB
#define IAP_GPIO_PIN GPIO_Pin_2
#define IAP_GPIO_LEVEL 1

#endif  // BOARD_TREX_V1_TYPE




//**********************************************************  config board hse clock

//config hse in "Options for Target" 
















//******************************************************** config iap 

#include "iap.h"









#endif
