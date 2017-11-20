#include "main_config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "uart.h"
#include "protocol.h"
#include "systick.h"
#include "iap.h"
#include "pwm.h"

#define _LOG(X...) if( 1 ) printf(X);



Uart_t Uart1;
Uart_t Uart2;
Uart_t Uart3;

#define CURRENT_UART &Uart3
#define PC_UART &Uart1
#define CONSOLE_UART &Uart1
#define IAP_UART &Uart1



#define RedLed_On() GPIO_WriteBit(GPIOB, GPIO_Pin_6, 0);
#define RedLed_Off() GPIO_WriteBit(GPIOB, GPIO_Pin_6, 1);
#define GreenLed_On() GPIO_WriteBit(GPIOB, GPIO_Pin_7, 0);
#define GreenLed_Off() GPIO_WriteBit(GPIOB, GPIO_Pin_7, 1);



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

	Uart_Configuration (&Uart1, USART1, 57600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
/*
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
*/
	
	//swd
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	Uart_config_console( CONSOLE_UART );
	if( BOARD_HAS_IAP )
		iap_init_in_uart( IAP_UART );

}





void pwm_output_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//gpio  B0(T1.2N) B1(T1.3N) B8(T4.3) B9(T4.4) C6(T3.1) C7(T3.2)  A7(T3.2)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
	//PB
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	pwm_init( TIM4, 3, 1000 , 50 , 1 );
	pwm_set( TIM4 , 3 , 1000/2 );
}



int pwm_capture_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	pwm_input( TIM3 , 4000, 20 , TIM_Channel_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
	
	return 0;
}

volatile uint16_t period,duty,last_period,last_duty;
volatile int pwm_capture_availed = 0;

void TIM3_IRQHandler(void)
 {

	 period = TIM_GetCapture2(TIM3);
	 if( SET == TIM_GetITStatus(TIM3, TIM_IT_CC2 ) ){
		if( period > 0 ){
			duty = TIM_GetCapture1(TIM3);
			period += last_period;
			duty += last_duty;
			pwm_capture_availed = 1;
			last_period = 0;
			last_duty = 0;
		}else{
			//break here
			period = 0;
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	 }else if( SET == TIM_GetITStatus(TIM3 , TIM_IT_Update) ){
		 last_period = period;
		 last_duty = TIM_GetCapture1(TIM3);
		 TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	 }else{
		 //break here
		 TIM_ClearITPendingBit(TIM3, 0xFF);
	 }
 }
 
 
void pwm_check_event()
{
	uint16_t diff;
	
	if( pwm_capture_availed ){
		pwm_capture_availed = 0;
		_LOG("pwm: %d,%d\n", period,duty);
		diff = duty*2 - period;
		if( abs( diff ) < 40 ){
			GreenLed_On();
		}else{
			GreenLed_Off();
		}
	}
}
 




void gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB , GPIO_InitStructure.GPIO_Pin);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA , GPIO_InitStructure.GPIO_Pin);
	
	
	// led pass false
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB , GPIO_InitStructure.GPIO_Pin);
	
	RedLed_On();
}

void numberled_toggle()
{
	static int step = 0;
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_4 );
	switch ( step ){
		case 0:
			GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15);
			GPIO_SetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_4 );
			break;
		case 1:
			GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15 );
			GPIO_SetBits(GPIOB,GPIO_Pin_15);
			break;
		case 2:
			//GPIO_SetBits(GPIOA,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4  | GPIO_Pin_6 | GPIO_Pin_8);
			GPIO_SetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_4 );
			break;
		case 3:
			GPIO_ResetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_15);
			GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_4 );
			break;
		default:
			step = 0;
			break;
		
	}
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	step++;
	step %= 4;
	
}

void barled_toggle()
{
	static int step = 0;
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 );
	
	switch ( step ){
		case 0:
			GPIO_SetBits(GPIOA,GPIO_Pin_0 );
			break;
		case 1:
			GPIO_SetBits(GPIOA,GPIO_Pin_1 );
			break;
		case 2:
			GPIO_SetBits(GPIOA,GPIO_Pin_2 );
			break;
		case 3:
			GPIO_SetBits(GPIOA,GPIO_Pin_3 );
			break;
		case 4:
			GPIO_SetBits(GPIOA,GPIO_Pin_4 );
			break;
		default:
			step = 0;
			break;
		
	}
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	step++;
	step %= 5;
}






static int sw_value;
static int sw_counter;
static int sw_on_tag;
static int sw_pressed;
#define SW_MAX_COUNT  10  // 10ms检查一次GPIO口， 10次就是100ms, 用作过滤
systick_time_t sw_timer;


void switch_det_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	sw_value = 0;
	sw_counter = 0;
	sw_on_tag = 0;
	sw_pressed = 0;
	
	systick_init_timer( &sw_timer, 10 );
}

void switch_det_event()
{
	
	if( 0 == systick_check_timer( &sw_timer ) )
		return;
	
	sw_value +=  GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_5 );
	sw_counter++;
	
	if( sw_counter >= SW_MAX_COUNT ){

		//if( sw_value >= SW_MAX_COUNT ){
		if( sw_value == 0 ){
			sw_on_tag = 1;
		}else {
			sw_on_tag = 0;
		}
		sw_value = 0;
		sw_counter = 0;
		
		
		if( sw_pressed == 0 ){
			sw_pressed = sw_on_tag;
		}else{
			if( sw_on_tag == 0 ){
				sw_pressed = 2;
				//or
				//sw_pressed = 0;
				//sw_press_handler();
			}
		}
	}
}

int switch_is_pressed()
{
	if( sw_pressed == 2 ){
		sw_pressed = 0;
		return 1;
	}
	return 0;
}

int switch_is_on()
{
	return sw_on_tag;
}






volatile int suspend_mode = 0;
volatile int key_press = 0;

void EXTI_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;  
	NVIC_InitTypeDef NVIC_InitStructure;	

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);

	EXTI_InitStructure.EXTI_Line = EXTI_Line5;    
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void enter_sleep()
{
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);//进入停止模式
}

void EXTI9_5_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line5);
		key_press = 1;
		//return;
		if( suspend_mode == 1 ){
			SystemInit();//时钟初始化
			suspend_mode = 0;
			return;
		}else{
			suspend_mode = 1;
			enter_sleep();
		}
	}
}

int is_key_press()
{
	if( key_press == 1 ){
		key_press = 0;
		return 1;
	}
	return 0;
}






systick_time_t led_timer;

void app_init()
{
	gpio_init();
	Uart_init();
	pwm_output_init();
	pwm_capture_init();
	switch_det_init();
	
	systick_init_timer( &led_timer, 1000);
}


void app_event()
{
	pwm_check_event();
	
	if( 1 == systick_check_timer( &led_timer) ){
		numberled_toggle();
		barled_toggle();
	}
	
	switch_det_event();
	if( switch_is_pressed() ){
		//numberled_toggle();
		//barled_toggle();
	}
}

