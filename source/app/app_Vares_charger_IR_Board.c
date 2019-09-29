#include "main_config.h"
#include "stm32f10x_conf.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"
#include "hw_config.h"
#include "flashRW.h"
#include "Pmsg.h"
#include "switcher.h"
#include "miniMeter.h"
#include "i2c.h"


#if BOARD_VARES_CHARGER_IR_BOARD

volatile int debug_en = 1;
#define _LOGW(X...) if( debug_en ) { printf("Warn: %s -> ",__FUNCTION__); printf(X);}
#define _LOGE(X...) printf("ERROR: %s -> ",__FUNCTION__); printf(X)
#define _LOGI(X...) if( debug_en ) { printf("Log: "); printf(X);}




Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;


#define CONSOLE_UART &Uart1
#define CONSOLE_CMD_MAX_LEN 32
unsigned char cmd_buffer[CONSOLE_CMD_MAX_LEN];



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
	
#if 0
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
	
	Uart_Configuration (&Uart2, USART2, 9600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
#endif

#if 0
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
	
	Uart_Configuration (&Uart3, USART3, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#endif
	
#if 0
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
	
	Uart_Configuration (&Uart5, UART5, 9600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
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
	console_cmd_config( cmd_buffer, CONSOLE_CMD_MAX_LEN);
}





#define SW_INTERVAL_COUNT  10  // SW_INTERVAL_MS 检查一次GPIO口， SW_INTERVAL_COUNT次后发出按键事件, 用作过滤
#define SW_INTERVAL_MS 5

#define IR_SIGNAL(X) GPIO_WriteBit( GPIOB, GPIO_Pin_8 , X )
#define IR_SIGNAL_LED(X) GPIO_WriteBit( GPIOC, GPIO_Pin_13 , (X==1?0:1) )

systick_time_t ir_timer;
volatile struct switcher irButton;
#define  IR_DELAY_US_TIMES (SystemCoreClock/1000000)
int IR_SIGNAL_PERIOD_DIV2_TIMES ;
volatile int IR_cycleSize = 20000;
volatile int IR_cycleCnt = 0;

void ir_delay_us( int ticks )
{
	volatile int i = ticks;
	
	while( i-- ) __NOP();
}

void switch_key_release_handler()
{
	int i;
	
	IR_SIGNAL_LED(1);
	
#if 1	
	__disable_irq();
	for( i=0; i<IR_cycleSize; i++ ){
	//while(1){
		IR_SIGNAL(1);
		ir_delay_us(IR_SIGNAL_PERIOD_DIV2_TIMES);
		IR_SIGNAL(0);
		ir_delay_us(IR_SIGNAL_PERIOD_DIV2_TIMES);
	}
	__enable_irq();
	
	systick_delay_ms(300);
	IR_SIGNAL_LED(0);
	
#else
		TIM_Cmd(TIM4, ENABLE);
		pwm_set( TIM4 , 3 , 30 );
#endif

	_LOGI("IR Signal KEY pressed\n");
}


void TIM4_IRQHandler()
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) 
	{
		IR_cycleCnt++;
		if( IR_cycleCnt >= IR_cycleSize ){
			TIM_Cmd(TIM4, DISABLE);
			IR_cycleCnt = 0;
			IR_SIGNAL_LED(0);
		}
		
	}else{
		TIM_ClearITPendingBit(TIM4, 0xff);
	}
}




void ir_output_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//gpio  B0(T1.2N) B1(T1.3N) B8(T4.3) B9(T4.4) C6(T3.1) C7(T3.2)  A7(T3.2)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
	
#if 1
	//PB
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	IR_SIGNAL(0);
	IR_SIGNAL_LED(0);
	IR_SIGNAL_PERIOD_DIV2_TIMES = 88;//IR_DELAY_US_TIMES; // 600us = 6 period = 1 signal;  1 period = 600us/6 ;  pwm duty = period/2 = 50us

#else 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	pwm_init( TIM4, 3, 60 , 38000 , 1, 0);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM4, DISABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel                        = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority      = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority             = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                     = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	
#endif
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	switcher_init( &irButton, SW_INTERVAL_COUNT, 1 , 0 , GPIOA, GPIO_Pin_8, GPIO_Mode_IN_FLOATING, switch_key_release_handler,NULL );
	systick_init_timer( &ir_timer, SW_INTERVAL_MS );
}
void ir_output_even()
{
	if( 0 == systick_check_timer( &ir_timer ) ) return;
	switcher_interval_check( &irButton );
}




void cmd_even()
{
	unsigned int i,j, len;
	
	len = console_cmd_check();
	
	if( len == 0 ) return;
	
	if( len == 4 && cmd_buffer[0] == 'c' && cmd_buffer[1] == '=')
	{
		cmd_buffer[len] = 0;
		sscanf((char*)cmd_buffer,"c=%d",&i);
		IR_cycleSize = i;
		_LOGI(" IR_cycleSize = %d \n", IR_cycleSize);
	}
}



void app_init()
{
	Uart_USB_SWJ_init();
	ir_output_init();

	if( SystemCoreClock != 72000000 ){
		_LOGE("CLK init error");
		systick_delay_ms(1000);
		while(1);
	}
	
	_LOGI("Init OK");
}


void app_event()
{
	cmd_even();
	ir_output_even();
}

#endif //BOARD_CSWL_LED_MONITOR
