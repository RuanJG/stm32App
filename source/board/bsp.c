#include "stm32f10x_conf.h"
#include "main_config.h"
#include "iap.h"
#include "uart.h"
#include "can1.h"
#include "bsp.h"
#include "systick.h"
#include "hw_config.h"



Uart_t *pConsoleUart = CONSOLE_NONE_TYPE;
volatile int console_type=0 ;

void console_init(int type, void * pridata )
{
	if( type == CONSOLE_USB_TYPE  && BOARD_USING_USB == 1 ){
		console_type |= type;
	}
	
	if( (type == CONSOLE_UART_TYPE) && (pridata != NULL) ){
		console_type |= type;
		pConsoleUart = (Uart_t*) pridata;
	}
}

int fputc(int ch, FILE *f)
{
	//TODO lock ?
	
	if( (console_type & CONSOLE_UART_TYPE) != 0   &&   pConsoleUart != NULL ){
		Uart_Put( pConsoleUart, (unsigned char*)&ch, 1);
	}
	
	if( (console_type & CONSOLE_USB_TYPE) != 0){
		USB_TxWrite( (unsigned char*)&ch, 1);
	}
	return (ch);
}




void bsp_init()
{
	
#if BOARD_HAS_IAP
	iap_config_vect_table();
#endif
	
	NVIC_PriorityGroupConfig(CUSTOM_SYSTICK_IRQ_PRIORITY);
	
	//if no this setting , flash will very slow
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	
#if BOARD_USING_SYSTICK
	systick_init(BOARD_SYSTICK_FREQ); 
#endif

#if BOARD_USING_USB
	USB_Config();
#endif
	
}

void bsp_event()
{
#if BOARD_USING_SYSTICK
	systick_event();
#endif
	
}

void bsp_deinit()
{
		//NO necessary now
}






