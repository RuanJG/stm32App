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

#define CUSTOM_CAN1_IRQ_PREPRIORITY 2
#define CUSTOM_CAN1_IRQ_SUBPRIORITY 0

#define CUSTOM_DAM1_IRQ_PREPRIORITY 2
#define CUSTOM_DAM1_IRQ_SUBPRIORITY 0




//**********************************************************  config board hse clock

//config hse in "Options for Target" 





//******************************************************** config iap 

//check config in  "iap.h"

#define BOARD_HAS_IAP  0
#define IAP_PORT_UART  1
#define IAP_PORT_CAN1  0  // not support now







#endif
