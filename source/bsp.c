#include "stm32f10x_conf.h"
#include "main_config.h"
#include "hw_config.h"


void gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	
#if IAP_PORT_UART	
	//USART1								  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = UART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(UART_TX_GPIO, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = UART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(UART_RX_GPIO, &GPIO_InitStructure); 
	
	UART_PIN_REMAP_FUNC();
#endif


#if  IAP_PORT_CAN1
	//CAN
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
									
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
#endif

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
}




int iap_first_check()
{
#if IAP_GPIO_DETECTION
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = IAP_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IAP_GPIO, &GPIO_InitStructure);
	
	GPIO_ReadInputDataBit( IAP_GPIO, IAP_GPIO_PIN);
	if( IAP_GPIO_LEVEL == GPIO_ReadInputDataBit( IAP_GPIO, IAP_GPIO_PIN) )
		return 1;
#endif 
	
	if( is_iap_tag_set() )
		return 1;
		
	return 0;
	
}


void bsp_init()
{
	NVIC_PriorityGroupConfig(CUSTOM_SYSTICK_IRQ_PRIORITY);
	
	//if no this setting , flash will very slow
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	
	if( 0 ==  iap_first_check() )
		iap_jump_to_app_or_deamon();
	
	gpio_init();
	
	systick_init();
	
	#if IAP_PORT_UART
		Uart_Configuration();
	#endif

	#if IAP_PORT_CAN1
		Can1_Configuration_mask(0, CAN1_ID, CAN_ID_STD, 0x1ff , CAN_SJW_1tq, CAN_BS1_3tq, CAN_BS2_5tq, 4);
	#endif
	
	#if  IAP_PORT_USB
		USB_Config();
	#endif
	

}

void bsp_event()
{
	//can run something like i2c spi event, here I run systick event
	systick_event();
	
}

void bsp_deinit()
{
	#if IAP_PORT_CAN1 
		CAN_DeInit(CAN1);
	#endif
	
	#if IAP_PORT_UART
		USART_Cmd(UARTDEV, DISABLE);	
	#endif
	
	#if IAP_PORT_USB
		USB_Deinit();
	#endif
	
	systick_deinit();
}
