#ifndef _MAIN_CONFIG_H
#define _MAIN_CONFIG_H

#include "stm32f10x.h"


//********************************************************** config irq priority

#define CUSTOM_SYSTICK_IRQ_PRIORITY 	NVIC_PriorityGroup_2

#define CUSTOM_UART1_IRQ_PREPRIORITY 2
#define CUSTOM_UART1_IRQ_SUBPRIORITY 0
#define CUSTOM_UART2_IRQ_PREPRIORITY 2
#define CUSTOM_UART2_IRQ_SUBPRIORITY 0
#define CUSTOM_UART3_IRQ_PREPRIORITY 2
#define CUSTOM_UART3_IRQ_SUBPRIORITY 0

#define CUSTOM_CAN1_IRQ_PREPRIORITY 1
#define CUSTOM_CAN1_IRQ_SUBPRIORITY 0

#define CUSTOM_DAM1_IRQ_PREPRIORITY 2
#define CUSTOM_DAM1_IRQ_SUBPRIORITY 0






//#####################################################   choose board

#define BOARD_IR_TESTBOARD	0
#define BOARD_MOTOR 				0
#define BOARD_QIPAD					0
#define BOARD_Trex_V2				1
#define BOARD_MOTOR_COOPERATE 0



//#####################################################   Config

//-----------------------Common Config

//**********************************************************  config board hse clock
//config hse in "Options for Target" 
//STM32F10X_MD,HSE_VALUE=8000000

//******************************************************** config iap 
//check config in  "iap.h"
#define BOARD_HAS_IAP  1
#define IAP_PORT_UART  0
#define IAP_PORT_CAN1  0  // not support now
#define IAP_PORT_USB   1

//**********************************************************  config using usb/can1
#define BOARD_USING_USB 1
#define USB_COM_RX_BUF_SIZE       512			//  (1024 + 256)
#define USB_COM_TX_BUF_SIZE       512			//  (1024 + 256)

//********************************************************* config can1
#define BOARD_USING_CAN1 0   // can1 and  usb  can't use in the sametime






//------------------------Board special Config

#if BOARD_Trex_V2

//******** config iap 
#define BOARD_HAS_IAP  1
#define IAP_PORT_UART  1
#define IAP_PORT_CAN1  0  // not support now
#define IAP_PORT_USB   1
//******** config usb(can1)
#define BOARD_USING_USB 1
#define USB_COM_RX_BUF_SIZE       512			//  (1024 + 256)
#define USB_COM_TX_BUF_SIZE       512			//  (1024 + 256)

#endif







#endif
