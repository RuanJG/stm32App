#include "stm32f10x_conf.h"
#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"
#include "hw_config.h"
#include "pwm.h"

#if BOARD_TSC3200



#define _LOG(X...) if( 1 ) printf(X);



Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;

//#define CONSOLE_UART &Uart3
#define PC_UART &Uart3
#define IAP_UART &Uart3
#define METER_UART &Uart1
#define CURRENT_UART &Uart1

void Uart_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	//USART1								  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    										  
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	Uart_Configuration (&Uart1, USART1, 9600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
#if 0
	//USART2								  
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
	
	//swd
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
}




#define S0(X) GPIO_WriteBit( GPIOB, GPIO_Pin_1 , X)
#define S1(X) GPIO_WriteBit( GPIOB, GPIO_Pin_0 , X)
#define S2(X) GPIO_WriteBit( GPIOA, GPIO_Pin_6 , X)
#define S3(X) GPIO_WriteBit( GPIOA, GPIO_Pin_5 , X)
#define E0(X) GPIO_WriteBit( GPIOA, GPIO_Pin_4 , X)

void tc3200_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	E0(1);
	S1(0);
	S2(0);
	S3(0);
	S0(0);
}



volatile unsigned char cmdbuff[32];
volatile int cmdLen = 0;
void tc3200_cmd_even()
{
	unsigned char buffer[8];
	unsigned char i, len;
	
	len = USB_RxRead( buffer, 8 );
	if( len == 0 ) return;
	for( i =0; i< len; i++)
	{
		cmdbuff[cmdLen++] = buffer[i];
	}
	
	for( i =0; i< cmdLen; i++)
	{
		switch(cmdbuff[i])
		{
			case 'R':
				S3(0);S2(0);_LOG("Red\n");break;
			case 'G':
				S3(1);S2(1);_LOG("Green\n");break;
			case 'C':
				S3(0);S2(1);_LOG("Clear\n");break;
			case 'B':
				S3(1);S2(0);_LOG("Blue\n");break;
				
			case 'E':
				E0(0);S0(1);S1(1);_LOG("Enable 100%\n");break;
			case 'D':
				E0(1);S0(0);S1(0);_LOG("Disable\n");break;
			
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
				pwm_set(TIM3,2,(cmdbuff[i]-0x30)*100);break;
			case 'A':
				pwm_set(TIM3,2,10*100);break;
			
		}
	}
	cmdLen=0;
}




unsigned short mPreiod,mPrescal,mFreq;
volatile unsigned int periodCnt;
volatile unsigned int tc3200_freq;

void tc3200_freq_capture_init()
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    										  

//capture timer2~7 72Mhz , 
	mPreiod = 64000; mPrescal = 1; mFreq = SystemCoreClock/mPreiod/(mPrescal);
	TIM_TimeBaseStructure.TIM_Period = mPreiod -1 ; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler = mPrescal -1 ; 	//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter    = 0x0;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	TIM_Cmd(TIM2, DISABLE);
	
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
	
	// Enable the TIM2 global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel                        = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority      = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority             = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                     = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
// Sample timer 10ms;  72000000 / period(1000) / prescal(720)
	TIM_TimeBaseStructure.TIM_Period = 1000 -1 ; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler = 720 -1 ; 	//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM4, DISABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	// Enable the TIM4 global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel                        = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority      = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority             = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                     = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
// init
	periodCnt = 0;
	tc3200_freq = 0;
	
	
//start 
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}




void TIM4_IRQHandler(void)
{
	//_LOG("%d\n",periodCnt);
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) 
	{
		TIM_Cmd(TIM2, DISABLE);
		TIM_Cmd(TIM4, DISABLE);
		
		periodCnt = periodCnt>0 ? (periodCnt-1):0 ;
		tc3200_freq = periodCnt*100;
		_LOG("F=%d K\n",tc3200_freq/1000);
		
		periodCnt = 0;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		TIM2->SR = (uint16_t)~TIM_IT_CC3;
		TIM2->CNT = 0;
		TIM4->CNT = 0;
		TIM_Cmd(TIM2, ENABLE);
		TIM_Cmd(TIM4, ENABLE);
	}else{
		TIM_ClearITPendingBit(TIM4, 0xff);
	}
}



void TIM2_IRQHandler(void)
{ 
	//if(TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET) 
	//{
	//TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
	TIM2->SR = (uint16_t)~TIM_IT_CC3;
	periodCnt++;
	//}
}

int tc3200_getFreq()
{
	return tc3200_freq;
}


systick_time_t tc3200_log_timer;

void app_init()
{
	tc3200_gpio_init();
	tc3200_freq_capture_init();
	systick_init_timer(&tc3200_log_timer, 200);

#if BOARD_HAS_IAP
	#if IAP_PORT_USB 
	iap_init_in_usb();
	#endif
#endif
	//console_init( CONSOLE_UART_TYPE ,CONSOLE_UART );
	console_init( CONSOLE_USB_TYPE ,NULL );
	heart_led_init();
	
	pwm_init( TIM3, 2, 1000, 1000 ,0 , 0);
	pwm_set(TIM3,2,0);
	
}





void app_event()
{
	if( systick_check_timer( &tc3200_log_timer ) ){
		_LOG("F=%d K\n",tc3200_getFreq()/1000);
		//_LOG("%d\n",periodCnt);
	}
	tc3200_cmd_even();
	heart_led_even();
}

#endif