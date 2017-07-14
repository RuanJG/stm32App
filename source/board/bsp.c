#include "stm32f10x_conf.h"
#include "main_config.h"
#include "iap.h"
#include "uart.h"
#include "can1.h"
#include "bsp.h"
#include "systick.h"


void bsp_init()
{
	
#if BOARD_HAS_IAP
	iap_config_vect_table();
#endif
	
	NVIC_PriorityGroupConfig(CUSTOM_SYSTICK_IRQ_PRIORITY);
	
	//if no this setting , flash will very slow
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	
	systick_init();
}

void bsp_event()
{
	//can run something like i2c spi event, here I run systick event
	systick_event();
	
}

void bsp_deinit()
{
	if ( 1== IAP_PORT_CAN1 )
		CAN_DeInit(CAN1);
	//if( 1 == IAP_PORT_UART)
		//USART_Cmd(UARTDEV, DISABLE);	
	systick_deinit();
}






