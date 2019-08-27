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


#if BOARD_CSWL_LED_MONITOR


volatile int debug_en = 1;
#define _LOGW(X...) if( debug_en ) { printf("Warn: %s",__FUNCTION__); printf(X);}
#define _LOGE(X...) printf("ERROR: %s",__FUNCTION__); printf(X)


PMSG_t PC_pmsg;

#define PC_TAG_LOGE  (PMSG_TAG_LOG|0x1)
#define PC_TAG_LOGI  (PMSG_TAG_LOG|0x2)

char logbuffer[ PROTOCOL_MAX_PACKET_SIZE ];
#define PC_LOG(T, X...) { snprintf( logbuffer, sizeof( logbuffer ), X);  PMSG_send_msg( &PC_pmsg , T , (unsigned char*)logbuffer, strlen(logbuffer) );}
#define PC_LOGI(X...) {PC_LOG( PC_TAG_LOGI , X);_LOGW(X);}
#define PC_LOGE(X...) {PC_LOG( PC_TAG_LOGE , X);_LOGE(X);}



//PC to LED
#define PC_TAG_CMD_CAPTURE_EN  (PMSG_TAG_CMD | 0x01 ) // data[0] = 1 auto start, data[0] = 0 stop
#define PC_TAG_CMD_LED_SELECT  (PMSG_TAG_CMD | 0x02 )  // data[0] = led id [1~12], data[1] = status
#define PC_TAG_CMD_CAPTURE_INTERVAL  (PMSG_TAG_CMD | 0x03 )  //data[0] = ms_L, data[1] = ms_H
#define PC_TAG_CMD_TEST (PMSG_TAG_CMD | 0x04 ) 
#define PC_TAG_CMD_SWITCHES_TESTMODE (PMSG_TAG_CMD | 0x05 ) // data[0] = 2,3,4,5 , FF mean off all
#define PC_TAG_CMD_SWITCHES_CHANNEL (PMSG_TAG_CMD | 0x06 ) // data[0] = 0~15 , FF mean off all
// PC->Control
#define PC_TAG_CMD_SWITCH  (PMSG_TAG_CMD | 0x07 ) // data[0] = SWITCH status
#define PC_TAG_CMD_UART_TEST  (PMSG_TAG_CMD | 0x08 ) // data[0] = Uart ID , 0 = stop


//LED to PC
#define PC_TAG_DATA_LED_BRIGHTNESS (PMSG_TAG_DATA|0x1)
//Control to PC
#define PC_TAG_DATA_VMETER (PMSG_TAG_DATA|0x2)
#define PC_TAG_DATA_AMETER (PMSG_TAG_DATA|0x3)
#define PC_TAG_DATA_BUTTON (PMSG_TAG_DATA|0x4) // startbutton data[0] = 1 pressed 





Uart_t Uart1;
Uart_t Uart3;
#define PC_UART &Uart1
#define IAP_UART &Uart1
#define CONSOLE_UART &Uart3






void Uart_USB_SWJ_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	//rempa after clock open and before pin config , otherwise PB3 PB4 will not work
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
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

	Uart_Configuration (&Uart1, USART1, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	
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














#define CAPTURE_IDEL			0
#define CAPTURE_STARTED		1
#define CAPTURE_FINISH		2
volatile unsigned int capture_time_ms	= 5 ;  // [ 0.1ms, 600ms ]
volatile unsigned int capture_1to6_tickers = 0;
volatile unsigned int capture_1to6_periods = 0;

volatile unsigned int capture_7to12_tickers = 0;
volatile unsigned int capture_7to12_periods = 0;

unsigned int capture_timer_period = 0xffff;
volatile int capture_counter_status = CAPTURE_IDEL;


void capture_counter_timer_init(TIM_TypeDef *timer)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	TIM_TimeBaseStructure.TIM_Period = capture_timer_period-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 1-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
	TIM_ITConfig(timer, TIM_IT_Update, ENABLE);
	TIM_ARRPreloadConfig(timer, DISABLE);
	
	//TIM_ITRxExternalClockConfig(timer,TIM_TS_ETRF);
	TIM_ETRClockMode2Config(timer, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
	//TIM_SelectInputTrigger(timer, TIM_TS_ETRF);
	
	TIM_ClearITPendingBit(timer, TIM_IT_Update);
	TIM_SetCounter(timer, 0);
	//TIM_Cmd(timer, ENABLE);
	
}

void cpature_counter_reset_interval( int ms)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	if( ms > 600 ) return ;
	
	TIM_Cmd(TIM3, DISABLE);
	TIM_TimeBaseStructure.TIM_Period = (100*ms)-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 720-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_ARRPreloadConfig(TIM3, DISABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	
	capture_time_ms = ms;
}

void capture_counter_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_0; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IN_FLOATING; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);	
	capture_counter_timer_init(TIM2);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	capture_counter_timer_init(TIM1);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	cpature_counter_reset_interval(capture_time_ms);
	
	NVIC_InitStructure.NVIC_IRQChannel                        = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority      = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority             = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                     = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	
	// Enable the Interrupt
	NVIC_InitStructure.NVIC_IRQChannel                        = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority      = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority             = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                     = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel                        = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority      = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority             = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                     = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update );
		capture_1to6_periods++;
	}
}

void TIM1_UP_IRQHandler()
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update );
		capture_7to12_periods++;
	}
}


void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{
		
		TIM_Cmd(TIM2, DISABLE);
		TIM_Cmd(TIM1, DISABLE);
		
		capture_1to6_tickers = TIM_GetCounter(TIM2);
		capture_7to12_tickers = TIM_GetCounter(TIM1); 
		capture_1to6_tickers += ( capture_1to6_periods * capture_timer_period );
		capture_7to12_tickers += ( capture_7to12_periods * capture_timer_period );
		capture_counter_status = CAPTURE_FINISH;
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update );
		TIM_Cmd(TIM3, DISABLE);
	}
	
}


void capture_counter_stop()
{
	TIM_Cmd(TIM3, DISABLE);
	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM1, DISABLE);
	capture_counter_status = CAPTURE_IDEL;
}

void capture_counter_start()
{
	capture_counter_stop();
	
	capture_1to6_tickers = 0;
	capture_1to6_periods = 0;
	capture_7to12_tickers = 0;
	capture_7to12_periods = 0;
	capture_counter_status = CAPTURE_STARTED;
	
	TIM_SetCounter(TIM2, 0);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_Cmd(TIM2, ENABLE);
	
	TIM_SetCounter(TIM1, 0);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	TIM_Cmd(TIM1, ENABLE);
	
	TIM_SetCounter(TIM3, 0);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_Cmd(TIM3, ENABLE);
}

int capture_counter_check(int *freq1_6, int *freq_7_12)
{
	if( capture_counter_status == CAPTURE_FINISH )
	{
		*freq1_6 = capture_1to6_tickers *1000 / capture_time_ms ;// HZ
		*freq_7_12 = capture_7to12_tickers *1000 / capture_time_ms ;// HZ
		capture_counter_stop();
		return 1;
	}
	return 0;
}








typedef enum _LED_ID {
	LED1 = 0,
	LED2,LED3,LED4,LED5,LED6,LED7,LED8,LED9,LED10,LED11,LED12	
}LED_ID_Type;

volatile unsigned char led_regs[12];
volatile unsigned char led_regs_feedback[12];
volatile unsigned char led_update_status;
volatile unsigned char led_current_id;

#define LED_STATUS_ENABLE			0x00
#define LED_STATUS_DISABLE		0x02
#define LED_STATUS_COLOR_RED				0x00
#define LED_STATUS_COLOR_GREEN			0x0C
#define LED_STATUS_COLOR_BLUE				0x04
#define LED_STATUS_COLOR_CLEAR				0x08
#define LED_STATUS_FREQ_100		0x30
#define LED_STATUS_FREQ_20		0x10
#define LED_STATUS_FREQ_2			0x20
#define LED_STATUS_FREQ_OFF			0x00
#define LED_STATUS_LED_ON				0x00
#define LED_STATUS_LED_OFF				0x01

#define LED_STATUS_OK  1
#define LED_STATUS_NG   0

#define LED_SERIAL_INPUT(X) GPIO_WriteBit( GPIOB, GPIO_Pin_4 , (X))
#define LED_SERIAL_nEN(X)   GPIO_WriteBit( GPIOB, GPIO_Pin_5 , (X))
#define LED_SERIAL_LOCK(X)  GPIO_WriteBit( GPIOB, GPIO_Pin_6 , (X))
#define LED_SERIAL_CLK(X)   GPIO_WriteBit( GPIOB, GPIO_Pin_7 , (X))
#define LED_SERIAL_REGCLR(X)   GPIO_WriteBit( GPIOB, GPIO_Pin_8 , (X))
#define LED_SERIAL_OUTPUT() GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)

#define LED_HAS_SERIAL_74HC595    1  // if using the 74HC595 


void _sn74hc595_delay_nop()
{
	volatile int i=36;
	while( i-- > 0 ) __NOP();
	//systick_delay_ms(1);
}

unsigned char bytes[84]={0};

void _sn74hc_regs()
{
	int i ,val,index,pos;
	unsigned char reg;
	unsigned char reg_feedback;
	
	for( index=0,i=LED12; i >= LED1; i--){
		for( pos = 0; pos < 6 ; pos ++) {
			bytes[index++] = ((led_regs[i]>>pos)&0x1);
		}
	}
}
	

void _sn74hc595_update_regs()
{
	int i ,val,index;
	unsigned char reg;
	unsigned char reg_feedback;
	
	
#if 1
	for( i = LED12; i >= LED1; i-- )
	{
		reg = led_regs[i];
		for( index = 0; index < 6 ; index++)
		{
			LED_SERIAL_INPUT( ((reg>>index) & 0x1) );
			LED_SERIAL_CLK(0);
		  _sn74hc595_delay_nop();
			LED_SERIAL_CLK(1);
			_sn74hc595_delay_nop();
		}
	}
#endif
	

	for( i = LED12; i >= LED1; i-- )
	{
		reg = led_regs[i];
		reg_feedback = 0;
		
		for( index = 0; index < 6 ; index++)
		{			

			val = LED_SERIAL_OUTPUT();
			reg_feedback = (reg_feedback | (val<<index)) ;
			
			LED_SERIAL_INPUT( ((reg>>index) & 0x1) );
			LED_SERIAL_CLK(0);
		  _sn74hc595_delay_nop();
			LED_SERIAL_CLK(1);
			_sn74hc595_delay_nop();

			#if 0
			LED_SERIAL_LOCK(0);
			_sn74hc595_delay_nop();
			LED_SERIAL_LOCK(1);
			#endif
		}
		led_regs_feedback[i] = reg_feedback;
		//PC_LOGE("0x%02x, %02x",reg_feedback,reg);
	}
	
}


/*
update the store reg
if return 0 sussefuly , -1 failed
*/
int catpure_select_update_led_status()
{
	int i,retry,res;
	
	if( 0 == LED_HAS_SERIAL_74HC595 ){
		// tsc3200 is control by hardware
		led_update_status = LED_STATUS_OK;
		return 0;
	}
	
	//clear shift reg
	//LED_SERIAL_REGCLR(0);
	//systick_delay_ms(1);
	//LED_SERIAL_REGCLR(1);
	
	//shift data
	retry  =  4; // at less 2 times
	while( 0 < retry-- )
	{
		_sn74hc595_update_regs();
		res = 1;
		for( i = LED1; i<= LED12; i++ )
		{
			if( led_regs[i] != led_regs_feedback[i] ){
				res = 0;
				PC_LOGE("_sn74hc595_update_regs retry");
				break;
			}
		}
		if( res == 1 ) break;
	}

	if( res == 1 )
	{
		//store to sotre reg
		LED_SERIAL_LOCK(0);
		_sn74hc595_delay_nop();
		LED_SERIAL_LOCK(1);
		// enable output
		LED_SERIAL_nEN(0); 
		
		led_update_status = LED_STATUS_OK;
	}else{
		led_update_status = LED_STATUS_NG;
	}
	
	return (res==1 ? 0:-1);
}

void capture_select_set_led_status(LED_ID_Type led, unsigned char status)
{
	led_regs[ led ] = status;
}


void capture_select_led_output( LED_ID_Type led )
{
	if( led >= LED1 && led <= LED6 )
	{
		GPIO_WriteBit( GPIOB, GPIO_Pin_0 , (led&0x1) );
		GPIO_WriteBit( GPIOB, GPIO_Pin_1 , (led&0x2)>>1 );
		GPIO_WriteBit( GPIOB, GPIO_Pin_2 , (led&0x4)>>2 );
	}else if ( led >= LED7 && led <= LED12 ){
		led -= 6 ;
		GPIO_WriteBit( GPIOA, GPIO_Pin_8 , (led&0x1) );
		GPIO_WriteBit( GPIOA, GPIO_Pin_7 , (led&0x2)>>1 );
		GPIO_WriteBit( GPIOA, GPIO_Pin_6 , (led&0x4)>>2 );
	}
}

//select led output and update the control pin of that tsc3200; successfully return 0, falied -1
int capture_select_led( LED_ID_Type led , unsigned char ctlparam)
{
	led_current_id = led;
	capture_select_led_output(led);
	if( ctlparam != led_regs[led] || led_update_status == LED_STATUS_NG ){
		capture_select_set_led_status(led,  ctlparam);
		return catpure_select_update_led_status();
	}
	return 0;
}

void catpure_select_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	int i;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	//74h151 , output select
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	capture_select_led_output(LED1);
	capture_select_led_output(LED7);
	
	
	
	//74h595 , led status control 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	LED_SERIAL_nEN(1); // high , output disabled
	LED_SERIAL_REGCLR(1); // low , shift reg clear
	LED_SERIAL_LOCK(0); // rise to triggle locking , so prepare now
	LED_SERIAL_CLK(0);
	
	for( i = LED1; i<= LED12; i++ ){
		capture_select_set_led_status( i,  LED_STATUS_ENABLE | LED_STATUS_COLOR_CLEAR | LED_STATUS_FREQ_100 | LED_STATUS_LED_OFF );
	}
	catpure_select_update_led_status();
	if ( LED_STATUS_NG ==  led_update_status){
		_LOGE("update 74hc595 reg failed\n");
	}
	
}







volatile unsigned int led_brightness[12];
volatile int auto_capture_index;
volatile int auto_capture_en;

void capture_reset_led_brightness()
{
	int i=0;
	for( i=0; i< 12; i++)
	{
		led_brightness[i] = 0;
		capture_select_set_led_status( i,  LED_STATUS_DISABLE | LED_STATUS_COLOR_CLEAR | LED_STATUS_FREQ_100 | LED_STATUS_LED_OFF );
	}
	auto_capture_index = LED1;
}


//tim 3 period , 0.1~500ms
void capture_set_interval( int ms ) 
{
	if( capture_counter_status != CAPTURE_IDEL ) {
		capture_counter_stop();
		cpature_counter_reset_interval( ms );
		capture_counter_start();
	}else{
		cpature_counter_reset_interval( ms );
	}
}

void capture_auto_mode()
{
	capture_counter_stop();
	capture_reset_led_brightness();
	auto_capture_en  = 1;
}

void capture_manual_mode()
{
	capture_counter_stop();
	auto_capture_en  = 0;
	capture_reset_led_brightness();
}

void capture_init()
{
	capture_reset_led_brightness();
	catpure_select_init();
	capture_counter_init();
	
	if ( LED_STATUS_NG ==  led_update_status){
		PC_LOGE("init 74hc595 reg failed\n");
	}
	capture_manual_mode();
}

void capture_loop()
{
	int freq1=0,freq2=0,res;
	
	if( auto_capture_en == 0) 
	{
		//manual mode  or falied select led
		if( 1 == capture_counter_check(&freq1, &freq2) )
		{
			if( led_current_id >= 0 && led_current_id< 6 ){
				led_brightness[led_current_id] = freq1;
			}else{
				led_brightness[led_current_id] = freq2;
			}
			PMSG_send_msg( &PC_pmsg, PC_TAG_DATA_LED_BRIGHTNESS , (unsigned char *)led_brightness , sizeof(led_brightness) ); 
			capture_counter_start();
		}
		return;
	}
	
	//check if get bringthness
	if( 1 == capture_counter_check(&freq1, &freq2) )
	{
		led_brightness[auto_capture_index] = freq1;
		led_brightness[auto_capture_index+6] = freq2;
		capture_select_set_led_status(auto_capture_index,  led_regs[auto_capture_index]| LED_STATUS_LED_OFF );
		capture_select_set_led_status(auto_capture_index+6,  led_regs[auto_capture_index+6]| LED_STATUS_LED_OFF );
		
		if( auto_capture_index >= LED6 ){
			auto_capture_index = LED1;
			PMSG_send_msg( &PC_pmsg, PC_TAG_DATA_LED_BRIGHTNESS , (unsigned char *)led_brightness , sizeof(led_brightness) ); 
		}else{
			auto_capture_index++;
		}
	}
	
	//capture_counter_check or the first time init , status = idel 
	if( capture_counter_status == CAPTURE_IDEL )
	{
		// init the tsc3200 
		res = capture_select_led(auto_capture_index , LED_STATUS_ENABLE | LED_STATUS_COLOR_CLEAR | LED_STATUS_FREQ_100 | LED_STATUS_LED_ON  ); // 1-6
		res += capture_select_led(auto_capture_index+6 , LED_STATUS_ENABLE | LED_STATUS_COLOR_CLEAR | LED_STATUS_FREQ_100 | LED_STATUS_LED_ON ); // 7-12
		if( res < 0 ) {
			// init led failed
			auto_capture_en = 0;
			PC_LOGE("init 74hc595 reg failed\n");
		}else{
			capture_counter_start();
		}
	}
}








//switchers control
#define SWITCHES_TESTMODE_ON_LEVEL 0
#define SWITCHES_TESTMODE_OFF_LEVEL 1
#define _SWITCHES_TESTMODE_S2(X) GPIO_WriteBit( GPIOB, GPIO_Pin_15 , (X==1? SWITCHES_TESTMODE_ON_LEVEL:SWITCHES_TESTMODE_OFF_LEVEL))
#define _SWITCHES_TESTMODE_S3(X) GPIO_WriteBit( GPIOA, GPIO_Pin_15 , (X==1? SWITCHES_TESTMODE_ON_LEVEL:SWITCHES_TESTMODE_OFF_LEVEL))
#define _SWITCHES_TESTMODE_S4(X) GPIO_WriteBit( GPIOA, GPIO_Pin_11 , (X==1? SWITCHES_TESTMODE_ON_LEVEL:SWITCHES_TESTMODE_OFF_LEVEL))
#define _SWITCHES_TESTMODE_Sn(X) GPIO_WriteBit( GPIOC, GPIO_Pin_15 , (X==1? SWITCHES_TESTMODE_ON_LEVEL:SWITCHES_TESTMODE_OFF_LEVEL))

void switches_testmode_S2()
{
	//_SWITCHES_TESTMODE_S3(0);
	//_SWITCHES_TESTMODE_S4(0);
	//_SWITCHES_TESTMODE_Sn(0);
	_SWITCHES_TESTMODE_S2(1);
}
void switches_testmode_S3()
{
	//_SWITCHES_TESTMODE_S4(0);
	//_SWITCHES_TESTMODE_Sn(0);
	//_SWITCHES_TESTMODE_S2(0);
	_SWITCHES_TESTMODE_S3(1);
}
void switches_testmode_S4()
{
	//_SWITCHES_TESTMODE_S3(0);
	//_SWITCHES_TESTMODE_Sn(0);
	//_SWITCHES_TESTMODE_S2(0);
	_SWITCHES_TESTMODE_S4(1);
}
void switches_testmode_Sn()
{
	//_SWITCHES_TESTMODE_S3(0);
	//_SWITCHES_TESTMODE_S4(0);
	//_SWITCHES_TESTMODE_S2(0);
	_SWITCHES_TESTMODE_Sn(1);
}

void switches_testmode_off()
{
	_SWITCHES_TESTMODE_S3(0);
	_SWITCHES_TESTMODE_S4(0);
	_SWITCHES_TESTMODE_S2(0);
	_SWITCHES_TESTMODE_Sn(0);
}


#define SWITCHES_CHANNEL_ON_LEVEL 0
#define SWITCHES_CHANNEL_OFF_LEVEL 1
#define SWITCHES_S0(X) GPIO_WriteBit( GPIOA, GPIO_Pin_1 ,(X==1? SWITCHES_CHANNEL_ON_LEVEL:SWITCHES_CHANNEL_OFF_LEVEL))
#define SWITCHES_S1(X) GPIO_WriteBit( GPIOA, GPIO_Pin_2 , (X==1? SWITCHES_CHANNEL_ON_LEVEL:SWITCHES_CHANNEL_OFF_LEVEL))
#define SWITCHES_S2(X) GPIO_WriteBit( GPIOA, GPIO_Pin_4 , (X==1? SWITCHES_CHANNEL_ON_LEVEL:SWITCHES_CHANNEL_OFF_LEVEL))
#define SWITCHES_S3(X) GPIO_WriteBit( GPIOA, GPIO_Pin_5 ,( X==1? SWITCHES_CHANNEL_ON_LEVEL:SWITCHES_CHANNEL_OFF_LEVEL))
#define SWITCHES_EN(X) GPIO_WriteBit( GPIOA, GPIO_Pin_3 , (X==1? SWITCHES_CHANNEL_ON_LEVEL:SWITCHES_CHANNEL_OFF_LEVEL))

int switches_channel_select(int channel)
{
	if( channel > 15 )
		return -1;
	
	SWITCHES_EN(1);
	systick_delay_ms(1);
	SWITCHES_S0(((channel>>0)&0x1));
	SWITCHES_S1(((channel>>1)&0x1));
	SWITCHES_S2(((channel>>2)&0x1));
	SWITCHES_S3(((channel>>3)&0x1));
	systick_delay_ms(1);
	SWITCHES_EN(0);
	return 0;
}

void switches_channel_off()
{
	SWITCHES_EN(1);
}


void switches_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	int i;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO ,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	//for PA15
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_11 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	switches_channel_off();
	
	switches_testmode_off();

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
				if( msg.data[0] == 0 ){
					capture_manual_mode();
				}else{
					capture_auto_mode();
				}
			}else{
				PC_LOGE("PC_TAG_CMD_CAPTURE_EN: unknow data");
			}
		break;
		
		case PC_TAG_CMD_LED_SELECT:
			capture_manual_mode();
			capture_reset_led_brightness();
			res = capture_select_led(msg.data[0]-1 , LED_STATUS_ENABLE | LED_STATUS_COLOR_CLEAR | LED_STATUS_FREQ_100 | LED_STATUS_LED_ON  );
			if( res == 0 ){
				PC_LOGI("PC_TAG_CMD_LED_SELECT: led%d start",msg.data[0]);
				capture_counter_start();
			}else{
				PC_LOGE("PC_TAG_CMD_LED_SELECT: failed");
			}
		break;
		
		case PC_TAG_CMD_CAPTURE_INTERVAL:
			if( msg.data_len == 2 ){
				value = msg.data[0] | (msg.data[1]<<8) ;
				capture_set_interval( value );
				PC_LOGI("set capture interval %dms",value);
			}else{
				PC_LOGE("PC_TAG_CMD_CAPTURE_INTERVAL:  unknow data");
			}
		break;
			
		case PC_TAG_CMD_TEST:
			//PC_LOGI("reg=0x%02x",catpure_select_test(msg.data[0]));
		break;
		
		case PC_TAG_CMD_SWITCHES_TESTMODE:
			if( msg.data_len ==1 ){
				if( msg.data[0] == 0xff ){ 
					switches_testmode_off();
					PC_LOGI("switches testmode off");
				}else{
					switch ( msg.data[0] ){
						case 2:
							switches_testmode_S2();PC_LOGI("switches testmode S2");break;
						case 3:
							switches_testmode_S3();PC_LOGI("switches testmode S3");break;
						case 4:
							switches_testmode_S4();PC_LOGI("switches testmode S4");break;
						case 5:
							switches_testmode_Sn();PC_LOGI("switches testmode Sn");break;
						default:
							PC_LOGE("PC_TAG_CMD_SWITCHES_TESTMODE:  testmode should be in [2,5]");
						break;
					}
				}
			}else{
				PC_LOGE("PC_TAG_CMD_SWITCHES_TESTMODE:  unknow data");
			}
		break;
		
		case PC_TAG_CMD_SWITCHES_CHANNEL:
			if( msg.data_len ==1 ){
				if( 0 > switches_channel_select(msg.data[0]) ){
					PC_LOGE("PC_TAG_CMD_SWITCHES_CHANNEL:  should be 0~15");
				}else{
					PC_LOGI("PC_TAG_CMD_SWITCHES_CHANNEL: channel %d ", msg.data[0]);
				}
			}else{
				PC_LOGE("PC_TAG_CMD_SWITCHES_CHANNEL:  unknow data");
			}
		break;
			
		default:
			break;
	}
	
}



void app_init()
{
	switches_init();
	Uart_USB_SWJ_init();
	PMSG_init_uart( &PC_pmsg, PC_UART, PC_msg_handler);
	capture_init();
	
	if( SystemCoreClock != 72000000 ){
		while(1);
	}
	
	PC_LOGI("Init OK");
}




volatile int led = 0;
void app_event()
{
	capture_loop();
	PMSG_even(&PC_pmsg);
	cmd_even();
}

#endif //BOARD_CSWL_LED_MONITOR
