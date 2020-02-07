#ifndef _MAIN_CONFIG_H
#define _MAIN_CONFIG_H

#include "stm32f10x.h"


//#####################################################   choose board && config MCU type, hse clock

//!!!!!!!    in "Options for Target" , like "STM32F10X_MD,HSE_VALUE=8000000 " 
/* #define STM32F10X_LD */     /*!< STM32F10X_LD: STM32 Low density devices */
/* #define STM32F10X_LD_VL */  /*!< STM32F10X_LD_VL: STM32 Low density Value Line devices */  
/* #define STM32F10X_MD */     /*!< STM32F10X_MD: STM32 Medium density devices */
/* #define STM32F10X_MD_VL */  /*!< STM32F10X_MD_VL: STM32 Medium density Value Line devices */  
/* #define STM32F10X_HD */     /*!< STM32F10X_HD: STM32 High density devices */
/* #define STM32F10X_HD_VL */  /*!< STM32F10X_HD_VL: STM32 High density value line devices */  
/* #define STM32F10X_XL */     /*!< STM32F10X_XL: STM32 XL-density devices */
/* #define STM32F10X_CL */     /*!< STM32F10X_CL: STM32 Connectivity line devices */

#define BOARD_IR_TESTBOARD			0  //STM32F10X_MD,HSE_VALUE=8000000
#define BOARD_MOTOR 						0  //STM32F10X_MD,HSE_VALUE=8000000
#define BOARD_QIPAD							0  //STM32F10X_MD,HSE_VALUE=8000000
#define BOARD_Trex_V2						0  //STM32F10X_MD,HSE_VALUE=8000000
#define BOARD_MOTOR_COOPERATE 	0  //STM32F10X_MD,HSE_VALUE=8000000
#define BOARD_MULTIKILL_PCBA 		0  //STM32F10X_MD,HSE_VALUE=8000000
#define BOARD_UC_SHAVER_SAMPLE 	0  //STM32F10X_MD,HSE_VALUE=8000000
#define BOARD_TSC3200 					0  //STM32F10X_MD,HSE_VALUE=8000000
#define BOARD_CSWL_LED_MONITOR 	0  //STM32F10X_MD,HSE_VALUE=16000000     //  0x8003C00 0xC400 ,
#define BOARD_CSWL_CONTROL_BOARD 	0  //STM32F10X_HD,HSE_VALUE=16000000   //STM32F103RCT6  0x8004000 0x3C000 , 
#define BOARD_CSWL_CONTROL_BOARD_V2 1  //STM32F10X_HD,HSE_VALUE=16000000   //STM32F103RCT6  0x8004000 0x3C000 , 
#define BOARD_VARES_CHARGER_BOARD 0  //STM32F10X_LD_VL,HSE_VALUE=0,BSP_TARGET_CLK=24000000
#define BOARD_VARES_CHARGER_IR_BOARD 0  //STM32F10X_MD,HSE_VALUE=8000000
#define BOARD_STUSB4500_PROGRAM_BOARD 0  //STM32F10X_MD,HSE_VALUE=8000000
#define BOARD_SWD_PROGRAMER 	0  //STM32F10X_MD,HSE_VALUE=8000000

//* when you want to build iap app, set BOARD_IAP 1 at the same time
#define BOARD_IAP			0   // board you choise(MCU type, hse clock)  0x8000000 0x10000 ,

#define IAP_APP_PORT_CAN1  0  
#define IAP_APP_PORT_USB   0
#define IAP_APP_PORT_UART 1



//#######################  IAP setup
// app addr : IAP_FIRMWARE_ADRESS + IAP_FIRMWARE_SIZE + FLASH_PAGE_SIZE  
// app size :(IAP_FLASH_SIZE - IAP_FIRMWARE_SIZE - FLASH_PAGE_SIZE) 
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)

#define FLASH_PAGE_SIZE    ((uint16_t)0x800)
#define IAP_FLASH_SIZE (0x40000) // Flash Totally size : you can get this value in 'Target Option' , when you choose your stm32 ic
#define IAP_FIRMWARE_ADRESS (0x8000000) // normal is 0x8000000
#define IAP_FIRMWARE_SIZE 0x3800 //14kB  ,  app addr = 0x8004000 0x3C000

#else

#define FLASH_PAGE_SIZE    ((uint16_t)0x400)
#define IAP_FLASH_SIZE (0x10000) // Flash Totally size : you can get this value in 'Target Option' , when you choose your stm32 ic
#define IAP_FIRMWARE_ADRESS (0x8000000) // normal is 0x8000000
#define IAP_FIRMWARE_SIZE 0x3800 //14kB ,  app addr = 0x8003C00 0xC400
	
#endif



//#######################  IAP App setup
/*
#define IAP_APP_UARTDEV		 	USART1
#define IAP_APP_UART_TX_GPIO 	GPIOA
#define IAP_APP_UART_TX_PIN 	GPIO_Pin_9 
#define IAP_APP_UART_RX_GPIO 	GPIOA
#define IAP_APP_UART_RX_PIN 	GPIO_Pin_10
#define IAP_APP_UART_PIN_REMAP_FUNC() //GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE)
#define IAP_APP_UART_BAUDRATE	115200
*/


//##################################################### config irq priority

#define CUSTOM_SYSTICK_IRQ_PRIORITY 	NVIC_PriorityGroup_2

#define CUSTOM_UART1_IRQ_PREPRIORITY 2
#define CUSTOM_UART1_IRQ_SUBPRIORITY 0
#define CUSTOM_UART2_IRQ_PREPRIORITY 2
#define CUSTOM_UART2_IRQ_SUBPRIORITY 0
#define CUSTOM_UART3_IRQ_PREPRIORITY 2
#define CUSTOM_UART3_IRQ_SUBPRIORITY 0
#define CUSTOM_UART4_IRQ_PREPRIORITY 2
#define CUSTOM_UART4_IRQ_SUBPRIORITY 0
#define CUSTOM_UART5_IRQ_PREPRIORITY 2
#define CUSTOM_UART5_IRQ_SUBPRIORITY 0

#define CUSTOM_CAN1_IRQ_PREPRIORITY 2
#define CUSTOM_CAN1_IRQ_SUBPRIORITY 0

#define CUSTOM_DAM1_IRQ_PREPRIORITY 2
#define CUSTOM_DAM1_IRQ_SUBPRIORITY 0




//#####################################################  Common Config 

//****** config iap ( more config in  "iap.h" )
#define BOARD_HAS_IAP  0
#define IAP_PORT_UART  0
#define IAP_PORT_CAN1  0  // not support now
#define IAP_PORT_USB   0


//******  config using usb  (can1 and  usb  can't use in the sametime )
#define BOARD_USING_USB 1
#define USB_COM_RX_BUF_SIZE       128			//  (1024 + 256)
#define USB_COM_TX_BUF_SIZE       128			//  (1024 + 256)

//****** config can1 (can1 and  usb  can't use in the sametime )
#define BOARD_USING_CAN1 0  

//****** config using systick for time 
#define BOARD_USING_SYSTICK 1
#define BOARD_SYSTICK_FREQ	1000

//****** board has special clk setup  , if all 0 , will use the system_init in lib
#define BOARD_COMMON_SETUP_CLK  1
#define BOARD_PRIVATE_SETUP_CLK  0
#ifndef BSP_TARGET_CLK
#define BSP_TARGET_CLK 72000000
#endif

//fput 
#define BOARD_SPECIAL_FPUT 0

//lib
#define LIB_STUSB4500_NVM  0

//module select
//#define USING_MINIMETER_MODULE




//#####################################################   Board special Config

#if BOARD_Trex_V2

//******** config iap 
#define BOARD_HAS_IAP  1
#define IAP_PORT_UART  1
#define IAP_PORT_USB   1
//******** config usb(can1)
#define BOARD_USING_USB 1
#define USB_COM_RX_BUF_SIZE       256			//  (1024 + 256)
#define USB_COM_TX_BUF_SIZE       256			//  (1024 + 256)

#endif //BOARD_UC_SHAVER_SAMPLE


#if BOARD_UC_SHAVER_SAMPLE

//******** config iap 
#define BOARD_HAS_IAP  1
#define IAP_PORT_UART  1
#define IAP_PORT_USB   1
//******** config usb(can1)
#define BOARD_USING_USB 1
#define USB_COM_RX_BUF_SIZE       256			//  (1024 + 256)
#define USB_COM_TX_BUF_SIZE       256			//  (1024 + 256)

#endif //BOARD_UC_SHAVER_SAMPLE



#if BOARD_CSWL_LED_MONITOR
#define BOARD_HAS_IAP  1
#define IAP_PORT_UART  0
#define IAP_PORT_USB   0
#define BOARD_USING_USB 0
#define BOARD_USING_SYSTICK 1
#define BOARD_COMMON_SETUP_CLK  1

#endif //BOARD_CSWL_LED_MONITOR

#if BOARD_CSWL_CONTROL_BOARD
#define BOARD_HAS_IAP  1
#define IAP_PORT_UART  0
#define IAP_PORT_USB   0
#define BOARD_USING_USB 0
#define BOARD_USING_SYSTICK 1
#define USING_MINIMETER_MODULE
#endif // BOARD_CSWL_CONTROL_BOARD

#if BOARD_CSWL_CONTROL_BOARD_V2
#define BOARD_HAS_IAP  1
#define IAP_PORT_UART  0
#define IAP_PORT_USB   0
#define BOARD_USING_USB 0
#define BOARD_USING_SYSTICK 1
#define USING_MINIMETER_MODULE
#endif // BOARD_CSWL_CONTROL_BOARD_V2


#if BOARD_VARES_CHARGER_BOARD
#define BOARD_HAS_IAP  0
#define BOARD_USING_USB 0
#define BOARD_USING_SYSTICK 1
#define BOARD_COMMON_SETUP_CLK  1
#define BOARD_PRIVATE_SETUP_CLK  0
#define BOARD_SPECIAL_FPUT 1
#endif // BOARD_VARES_CHARGER_BOARD

#if BOARD_STUSB4500_PROGRAM_BOARD
#define BOARD_HAS_IAP  0
#define BOARD_USING_USB 1
#define BOARD_USING_SYSTICK 1
#define LIB_STUSB4500_NVM 1
#endif // BOARD_STUSB4500_PROGRAM_BOARD

#if BOARD_VARES_CHARGER_IR_BOARD
#define BOARD_HAS_IAP  0
#define BOARD_USING_USB 1
#define BOARD_USING_SYSTICK 1
#endif // BOARD_VARES_CHARGER_IR_BOARD

#if BOARD_SWD_PROGRAMER
#define BOARD_HAS_IAP  0
#define IAP_PORT_UART  0
#define IAP_PORT_USB   0
#define BOARD_USING_USB 0
#define USB_COM_RX_BUF_SIZE       4			//  (1024 + 256)
#define USB_COM_TX_BUF_SIZE       4			//  (1024 + 256)
#define BOARD_USING_SYSTICK 0
#endif //BOARD_UC_SHAVER_SAMPLE



#endif
