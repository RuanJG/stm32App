#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"
#include "hw_config.h"
#include "flashRW.h"
#include "Pmsg.h"


#if BOARD_CSWL_CONTROL_BOARD



volatile int debug_en = 1;
#define _LOGW(X...) if( debug_en ) { printf("Warn: %s -> ",__FUNCTION__); printf(X);}
#define _LOGE(X...) printf("ERROR: %s -> ",__FUNCTION__); printf(X)


#define PC_TAG_LOGE  (PMSG_TAG_LOG|0x1)
#define PC_TAG_LOGI  (PMSG_TAG_LOG|0x2)

unsigned char logbuffer[ PROTOCOL_MAX_PACKET_SIZE ];
#define PC_LOG(T, X...) { snprintf( logbuffer, sizeof( logbuffer ), X);  PMSG_send_msg( T , logbuffer, strlen(logbuffer) );}
#define PC_LOGI(X...) {PC_LOG( PC_TAG_LOGI , X);_LOGW(X);}
#define PC_LOGE(X...) {PC_LOG( PC_TAG_LOGE , X);_LOGE(X);}


#define PC_TAG_CMD_CAPTURE_EN  (PMSG_TAG_CMD | 0x01 ) // data[0] = 1 auto start, data[0] = 0 stop
#define PC_TAG_CMD_LED_SELECT  (PMSG_TAG_CMD | 0x02 )  // data[0] = led id [1~12], data[1] = status
#define PC_TAG_CMD_CAPTURE_INTERVAL  (PMSG_TAG_CMD | 0x03 )  //data[0] = ms_L, data[1] = ms_H
#define PC_TAG_CMD_TEST (PMSG_TAG_CMD | 0x04 ) 
#define PC_TAG_CMD_SWITCHES_TESTMODE (PMSG_TAG_CMD | 0x05 ) // data[0] = 2,3,4,5 , FF mean off all
#define PC_TAG_CMD_SWITCHES_CHANNEL (PMSG_TAG_CMD | 0x06 ) // data[0] = 0~15 , FF mean off all


#define PC_TAG_DATA_LED_BRIGHTNESS (PMSG_TAG_DATA|0x1)






Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;
Uart_t Uart4;
//Uart_t Uart5;

#define PC_UART &Uart1
#define LED_UART &Uart4
#define IAP_UART &Uart1
#define CONSOLE_UART &Uart3


void Uart_USB_SWJ_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	//USART1								  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	Uart_Configuration (&Uart1, USART1, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#if 1
	//USART2				
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	Uart_Configuration (&Uart2, USART2, 57600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
#endif

#if 1
	//USART3		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	Uart_Configuration (&Uart3, USART3, 57600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#endif
	
#if 1
	//UART4					
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	Uart_Configuration (&Uart4, UART4, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#endif


#if 0
	//UART5				
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	Uart_Configuration (&Uart5, UART5, 57600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#endif
	
	
	
	
	
	
	
	//swd
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	
#if BOARD_HAS_IAP
	#if IAP_PORT_USB 
	iap_init_in_usb();
	#endif
	#if IAP_PORT_CAN1
	iap_init_in_can1();
	#endif
	#if  IAP_PORT_UART
	iap_init_in_uart( IAP_UART );
	#endif
#endif

	console_init( CONSOLE_UART_TYPE ,CONSOLE_UART );
	console_init( CONSOLE_USB_TYPE ,NULL );
	
}






#define RELAY_ON 0
#define RELAY_OFF 1

#define RELAY_OUTPUT_TO_SHAVE(X) GPIO_WriteBit( GPIOC, GPIO_Pin_5 , X==1? RELAY_ON:RELAY_OFF)
#define RELAY_OUTPUT_TO_METER(X) GPIO_WriteBit( GPIOC, GPIO_Pin_4 , X==1? RELAY_ON:RELAY_OFF)
#define RELAY_OUTPUT_TO_RESISTOR(X) GPIO_WriteBit( GPIOC, GPIO_Pin_6 , X==1? RELAY_ON:RELAY_OFF)
#define RELAY_VDD_TO_METER(X) GPIO_WriteBit( GPIOC, GPIO_Pin_7 , X==1? RELAY_ON:RELAY_OFF)
#define RELAY_POWER_TO_CSWL(X) GPIO_WriteBit( GPIOC, GPIO_Pin_8 , X==1? RELAY_ON:RELAY_OFF)

#define SIGNAL_ON 1
#define SIGNAL_OFF 0
#define SIGNAL_TO_AIRPRESS(X) GPIO_WriteBit( GPIOA, GPIO_Pin_1 , X==1? SIGNAL_ON:SIGNAL_OFF)

void relay_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SIGNAL_TO_AIRPRESS(0);
	
	RELAY_OUTPUT_TO_SHAVE(0);
	RELAY_OUTPUT_TO_METER(0);
	RELAY_OUTPUT_TO_RESISTOR(0);
	RELAY_VDD_TO_METER(0);
	RELAY_POWER_TO_CSWL(0);

}








void cmd_even()
{
	unsigned char buffer[8];
	unsigned char i, len;
	
	len = Uart_Get(CONSOLE_UART,buffer,1);// USB_RxRead( buffer, 8 );
	
	if( len == 0 ) return;

	if( buffer[0] == 'd' ){
		debug_en = debug_en ? 0:1;
	}else if( buffer[0] == '0' ){

	}
}





void PC_msg_handler(PMSG_msg_t msg)
{
	int value,res;
	
	switch( msg.tag )
	{
		case PC_TAG_CMD_CAPTURE_EN:
			if( msg.data_len == 1 ){

			}else{
				PC_LOGE("PC_TAG_CMD_CAPTURE_EN: unknow data");
			}
		break;
		
			
		default:
			PC_LOGE("PC_TAG_CMD_SWITCHES_CHANNEL:  unknow data");
			break;
	}
	
}



void app_SystemInit()
{
	if( BOARD_PRIVATE_SETUP_CLK == 1 ){
		bsp_stm32f103_SystemInit();
	}else{
		SystemInit();
	}
}


void app_init()
{
	relay_init();
	Uart_USB_SWJ_init();
	PMSG_init(PC_UART, PC_msg_handler , NULL , NULL );
	
	if( SystemCoreClock != 72000000 ){
		_LOGE("CLK init error");
		systick_delay_ms(100);
		while(1);
	}
	
	PC_LOGI("Init OK");
}


void app_event()
{

	PMSG_even();
	cmd_even();
}

#endif //BOARD_CSWL_LED_MONITOR
